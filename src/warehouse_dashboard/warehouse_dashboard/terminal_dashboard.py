"""
Terminal Dashboard -- Rich-based TUI for monitoring the warehouse robot.

Panels:
  - Robot Pose    (from /tf: base_footprint in map frame)
  - Battery       (gauge from /battery_state)
  - Current Task  (from /current_task)
  - Task Queue    (table from /task_queue)
  - Diagnostics   (from /diagnostics)

Updates at 2 Hz using rich.live.Live.
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped

from warehouse_interfaces.msg import DeliveryOrder as DeliveryOrderMsg
from warehouse_interfaces.msg import TaskQueue as TaskQueueMsg

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rich.console import Console
from rich.live import Live
from rich.table import Table
from rich.panel import Panel
from rich.layout import Layout
from rich.text import Text
from rich.progress_bar import ProgressBar


class DashboardNode(Node):
    def __init__(self):
        super().__init__('terminal_dashboard')

        # ---- Parameters --------------------------------------------------------
        self.declare_parameter('update_rate', 2.0)
        update_rate = self.get_parameter('update_rate').value

        # ---- Shared state (written by callbacks, read by display thread) -------
        self.lock = threading.Lock()
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.tf_ok = False

        self.battery_pct = 0.0
        self.battery_voltage = 0.0
        self.battery_status = 'UNKNOWN'
        self.battery_ok = False

        self.current_task = DeliveryOrderMsg()
        self.task_queue: list[DeliveryOrderMsg] = []

        self.diagnostics: list[tuple[str, int, str]] = []  # (name, level, message)

        # ---- TF2 ---------------------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- Subscribers -------------------------------------------------------
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        self.create_subscription(
            BatteryState, '/battery_state', self._battery_cb, 10)
        self.create_subscription(
            DeliveryOrderMsg, '/current_task', self._current_task_cb, qos_reliable)
        self.create_subscription(
            TaskQueueMsg, '/task_queue', self._task_queue_cb, qos_reliable)
        self.create_subscription(
            DiagnosticArray, '/diagnostics', self._diag_cb, 10)

        # ---- TF polling timer --------------------------------------------------
        self.create_timer(1.0 / update_rate, self._poll_tf)

        self.get_logger().info('Terminal Dashboard started.')

    # =========================================================================
    # Callbacks
    # =========================================================================
    def _battery_cb(self, msg: BatteryState):
        with self.lock:
            self.battery_pct = msg.percentage * 100.0
            self.battery_voltage = msg.voltage
            self.battery_ok = True
            status_map = {
                BatteryState.POWER_SUPPLY_STATUS_CHARGING: 'CHARGING',
                BatteryState.POWER_SUPPLY_STATUS_DISCHARGING: 'DISCHARGING',
                BatteryState.POWER_SUPPLY_STATUS_FULL: 'FULL',
                BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING: 'NOT CHARGING',
            }
            self.battery_status = status_map.get(msg.power_supply_status, 'UNKNOWN')

    def _current_task_cb(self, msg: DeliveryOrderMsg):
        with self.lock:
            self.current_task = msg

    def _task_queue_cb(self, msg: TaskQueueMsg):
        with self.lock:
            self.task_queue = list(msg.orders)

    def _diag_cb(self, msg: DiagnosticArray):
        with self.lock:
            self.diagnostics = [
                (s.name, s.level, s.message) for s in msg.status
            ]

    def _poll_tf(self):
        try:
            t: TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            q = t.transform.rotation
            # yaw from quaternion
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            with self.lock:
                self.robot_x = t.transform.translation.x
                self.robot_y = t.transform.translation.y
                self.robot_yaw = yaw
                self.tf_ok = True
        except TransformException:
            with self.lock:
                self.tf_ok = False

    # =========================================================================
    # Rendering
    # =========================================================================
    def render(self) -> Layout:
        with self.lock:
            pose_panel = self._render_pose()
            battery_panel = self._render_battery()
            task_panel = self._render_current_task()
            queue_panel = self._render_queue()
            diag_panel = self._render_diagnostics()

        layout = Layout()
        layout.split_column(
            Layout(name='top', size=5),
            Layout(name='middle', size=10),
            Layout(name='bottom'),
        )
        layout['top'].split_row(
            Layout(pose_panel, name='pose'),
            Layout(battery_panel, name='battery'),
        )
        layout['middle'].update(task_panel)
        layout['bottom'].split_row(
            Layout(queue_panel, name='queue'),
            Layout(diag_panel, name='diag'),
        )
        return layout

    def _render_pose(self) -> Panel:
        if self.tf_ok:
            text = (
                f'X: {self.robot_x:+8.2f} m\n'
                f'Y: {self.robot_y:+8.2f} m\n'
                f'Yaw: {math.degrees(self.robot_yaw):+7.1f} deg'
            )
        else:
            text = '[dim]Waiting for TF...[/dim]'
        return Panel(text, title='Robot Pose', border_style='cyan')

    def _render_battery(self) -> Panel:
        pct = self.battery_pct
        if not self.battery_ok:
            text = '[dim]No data[/dim]'
        else:
            if pct > 50:
                color = 'green'
            elif pct > 20:
                color = 'yellow'
            else:
                color = 'red'
            bar = f'[{color}]{"#" * int(pct // 5)}[/{color}]{"." * (20 - int(pct // 5))}'
            text = (
                f'{bar}  {pct:.1f}%\n'
                f'Voltage: {self.battery_voltage:.1f} V  |  {self.battery_status}'
            )
        return Panel(text, title='Battery', border_style='green')

    def _render_current_task(self) -> Panel:
        t = self.current_task
        if not t.order_id:
            text = '[dim]No active task -- robot idle[/dim]'
        else:
            text = (
                f'Order:    {t.order_id}\n'
                f'Pickup:   {t.pickup_station}\n'
                f'Delivery: {t.delivery_station}\n'
                f'Priority: {t.priority}\n'
                f'Status:   {t.status}'
            )
        return Panel(text, title='Current Task', border_style='magenta')

    def _render_queue(self) -> Panel:
        table = Table(show_header=True, header_style='bold')
        table.add_column('#', width=3)
        table.add_column('Order ID', width=10)
        table.add_column('From', width=14)
        table.add_column('To', width=14)
        table.add_column('Pri', width=4)
        table.add_column('Status', width=20)

        if not self.task_queue:
            table.add_row('-', '[dim]Queue empty[/dim]', '', '', '', '')
        else:
            for i, order in enumerate(self.task_queue):
                table.add_row(
                    str(i + 1),
                    order.order_id,
                    order.pickup_station,
                    order.delivery_station,
                    str(order.priority),
                    order.status,
                )
        return Panel(table, title='Task Queue', border_style='blue')

    def _render_diagnostics(self) -> Panel:
        if not self.diagnostics:
            text = '[dim]No diagnostics received[/dim]'
        else:
            lines = []
            level_map = {0: '[green]OK[/green]', 1: '[yellow]WARN[/yellow]',
                         2: '[red]ERROR[/red]', 3: '[red bold]STALE[/red bold]'}
            for name, level, message in self.diagnostics:
                lvl_str = level_map.get(level, str(level))
                lines.append(f'{lvl_str}  {name}: {message}')
            text = '\n'.join(lines)
        return Panel(text, title='Diagnostics', border_style='yellow')


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()

    # Spin ROS in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    console = Console()
    try:
        with Live(node.render(), console=console, refresh_per_second=2,
                  screen=True) as live:
            while rclpy.ok():
                live.update(node.render())
                import time
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
