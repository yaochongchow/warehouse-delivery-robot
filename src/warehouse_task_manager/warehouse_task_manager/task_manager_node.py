"""
Task Manager Node -- Main orchestration node for the warehouse delivery robot.

Maintains a priority queue of delivery orders and drives the robot through
a state machine: IDLE -> NAV_TO_PICKUP -> PICKING_UP -> NAV_TO_DELIVERY ->
DROPPING_OFF -> IDLE.  Monitors battery state and interrupts to navigate
to the charging station when charge drops below 20 %.
"""

import heapq
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from ament_index_python.packages import get_package_share_directory

from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped

from warehouse_interfaces.msg import DeliveryOrder as DeliveryOrderMsg
from warehouse_interfaces.msg import TaskQueue as TaskQueueMsg
from warehouse_interfaces.srv import SubmitOrder, CancelOrder

from diagnostic_updater import Updater, FunctionDiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus

from warehouse_task_manager.models import DeliveryOrder, TaskStatus
from warehouse_task_manager.stations import load_stations, resolve_station_name


class State:
    IDLE = 'IDLE'
    NAV_TO_PICKUP = 'NAV_TO_PICKUP'
    PICKING_UP = 'PICKING_UP'
    NAV_TO_DELIVERY = 'NAV_TO_DELIVERY'
    DROPPING_OFF = 'DROPPING_OFF'
    NAV_TO_CHARGER = 'NAV_TO_CHARGER'
    CHARGING = 'CHARGING'


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node')

        # ---- Parameters --------------------------------------------------------
        self.declare_parameter('stations_yaml', '')
        self.declare_parameter('low_battery_threshold', 20.0)
        self.declare_parameter('charge_resume_threshold', 80.0)
        self.declare_parameter('pickup_duration', 2.0)
        self.declare_parameter('dropoff_duration', 2.0)

        stations_yaml = self.get_parameter('stations_yaml').get_parameter_value().string_value
        if not stations_yaml:
            pkg_share = get_package_share_directory('warehouse_task_manager')
            stations_yaml = pkg_share + '/config/stations.yaml'

        self.low_battery = self.get_parameter('low_battery_threshold').value
        self.charge_resume = self.get_parameter('charge_resume_threshold').value
        self.pickup_duration = self.get_parameter('pickup_duration').value
        self.dropoff_duration = self.get_parameter('dropoff_duration').value

        # ---- Load stations -----------------------------------------------------
        self.stations = load_stations(stations_yaml)
        self.get_logger().info(
            f'Loaded {len(self.stations)} stations: {list(self.stations.keys())}'
        )

        # ---- State -------------------------------------------------------------
        self.state = State.IDLE
        self.task_queue: list = []          # heapq of (priority, DeliveryOrder)
        self.current_order: DeliveryOrder | None = None
        self.battery_pct: float = 100.0
        self.nav_goal_handle = None
        self.wait_start: float | None = None
        self.interrupted_order: DeliveryOrder | None = None

        # ---- Callback group for concurrent service / action handling -----------
        self.cb_group = ReentrantCallbackGroup()

        # ---- Nav2 action client ------------------------------------------------
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.cb_group,
        )

        # ---- Publishers --------------------------------------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )
        self.current_task_pub = self.create_publisher(DeliveryOrderMsg, '/current_task', qos)
        self.task_queue_pub = self.create_publisher(TaskQueueMsg, '/task_queue', qos)

        # ---- Subscribers -------------------------------------------------------
        self.create_subscription(
            BatteryState, '/battery_state', self._battery_cb, 10,
            callback_group=self.cb_group,
        )

        # ---- Services ----------------------------------------------------------
        self.create_service(
            SubmitOrder, '/submit_order', self._submit_order_cb,
            callback_group=self.cb_group,
        )
        self.create_service(
            CancelOrder, '/cancel_order', self._cancel_order_cb,
            callback_group=self.cb_group,
        )

        # ---- Diagnostics -------------------------------------------------------
        self.diag_updater = Updater(self)
        self.diag_updater.setHardwareID('task_manager')
        self.diag_updater.add(
            FunctionDiagnosticTask('Task Manager Status', self._diag_callback)
        )

        # ---- Timers ------------------------------------------------------------
        self.create_timer(1.0, self._publish_status)
        self.create_timer(0.2, self._tick_state_machine)

        self.get_logger().info('Task Manager Node started.')

    # =========================================================================
    # Service callbacks
    # =========================================================================
    def _submit_order_cb(self, request, response):
        pickup = resolve_station_name(request.pickup_station)
        delivery = resolve_station_name(request.delivery_station)

        if pickup not in self.stations:
            response.accepted = False
            response.message = f'Unknown pickup station: {request.pickup_station}'
            response.order_id = ''
            return response

        if delivery not in self.stations:
            response.accepted = False
            response.message = f'Unknown delivery station: {request.delivery_station}'
            response.order_id = ''
            return response

        order = DeliveryOrder(
            pickup_station=pickup,
            delivery_station=delivery,
            priority=int(request.priority) if request.priority else 1,
        )
        heapq.heappush(self.task_queue, (order.priority, order))
        self.get_logger().info(
            f'Order {order.order_id} accepted: {pickup} -> {delivery} (p={order.priority})'
        )

        response.order_id = order.order_id
        response.accepted = True
        response.message = 'Order submitted successfully.'
        return response

    def _cancel_order_cb(self, request, response):
        # Try to cancel from queue first
        for i, (_, order) in enumerate(self.task_queue):
            if order.order_id == request.order_id:
                order.status = TaskStatus.CANCELLED
                self.task_queue.pop(i)
                heapq.heapify(self.task_queue)
                response.success = True
                response.message = f'Order {request.order_id} cancelled from queue.'
                self.get_logger().info(response.message)
                return response

        # Cancel current order
        if self.current_order and self.current_order.order_id == request.order_id:
            self.current_order.status = TaskStatus.CANCELLED
            if self.nav_goal_handle is not None:
                self.nav_goal_handle.cancel_goal_async()
            self.current_order = None
            self.state = State.IDLE
            response.success = True
            response.message = f'Active order {request.order_id} cancelled.'
            self.get_logger().info(response.message)
            return response

        response.success = False
        response.message = f'Order {request.order_id} not found.'
        return response

    # =========================================================================
    # Battery callback
    # =========================================================================
    def _battery_cb(self, msg: BatteryState):
        self.battery_pct = msg.percentage * 100.0  # BatteryState uses 0.0-1.0

    # =========================================================================
    # State machine
    # =========================================================================
    def _tick_state_machine(self):
        # ---------- Low battery interrupt ------------------------------------
        if (self.battery_pct < self.low_battery
                and self.state not in (State.NAV_TO_CHARGER, State.CHARGING)):
            self.get_logger().warn(
                f'Low battery ({self.battery_pct:.1f}%), heading to charger.'
            )
            # Save current order so we can resume later
            if self.current_order is not None:
                self.interrupted_order = self.current_order
                self.current_order = None
                if self.nav_goal_handle is not None:
                    self.nav_goal_handle.cancel_goal_async()
                    self.nav_goal_handle = None
            self.state = State.NAV_TO_CHARGER
            self._send_nav_goal('charging')
            return

        # ---------- Charging state -------------------------------------------
        if self.state == State.CHARGING:
            if self.battery_pct >= self.charge_resume:
                self.get_logger().info('Battery charged, resuming operations.')
                # Re-enqueue interrupted order
                if self.interrupted_order is not None:
                    self.interrupted_order.status = TaskStatus.PENDING
                    heapq.heappush(
                        self.task_queue,
                        (self.interrupted_order.priority, self.interrupted_order),
                    )
                    self.interrupted_order = None
                self.state = State.IDLE
            return

        # ---------- IDLE -- pick next order ----------------------------------
        if self.state == State.IDLE:
            if not self.task_queue:
                return
            _, order = heapq.heappop(self.task_queue)
            self.current_order = order
            self.current_order.status = TaskStatus.NAVIGATING_TO_PICKUP
            self.state = State.NAV_TO_PICKUP
            self.get_logger().info(
                f'Starting order {order.order_id}: navigating to {order.pickup_station}'
            )
            self._send_nav_goal(order.pickup_station)

        # ---------- PICKING_UP / DROPPING_OFF (timed wait) -------------------
        elif self.state == State.PICKING_UP:
            if self.wait_start is None:
                self.wait_start = time.time()
                self.current_order.status = TaskStatus.PICKING_UP
                self.get_logger().info(
                    f'Picking up at {self.current_order.pickup_station}...'
                )
            elif time.time() - self.wait_start >= self.pickup_duration:
                self.wait_start = None
                self.current_order.status = TaskStatus.NAVIGATING_TO_DELIVERY
                self.state = State.NAV_TO_DELIVERY
                self.get_logger().info(
                    f'Pickup complete, navigating to {self.current_order.delivery_station}'
                )
                self._send_nav_goal(self.current_order.delivery_station)

        elif self.state == State.DROPPING_OFF:
            if self.wait_start is None:
                self.wait_start = time.time()
                self.current_order.status = TaskStatus.DROPPING_OFF
                self.get_logger().info(
                    f'Dropping off at {self.current_order.delivery_station}...'
                )
            elif time.time() - self.wait_start >= self.dropoff_duration:
                self.wait_start = None
                self.current_order.status = TaskStatus.COMPLETED
                self.get_logger().info(
                    f'Order {self.current_order.order_id} completed!'
                )
                self.current_order = None
                self.state = State.IDLE

    # =========================================================================
    # Navigation helpers
    # =========================================================================
    def _send_nav_goal(self, station_name: str):
        if station_name not in self.stations:
            self.get_logger().error(f'Station "{station_name}" not found in stations map.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        src = self.stations[station_name]
        goal_pose.header.frame_id = src.header.frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = src.pose
        goal_msg.pose = goal_pose

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            return

        self.get_logger().info(f'Sending navigation goal to station "{station_name}"')
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self._nav_feedback_cb
        )
        send_goal_future.add_done_callback(self._nav_goal_response_cb)

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal was rejected.')
            self._handle_nav_failure()
            return

        self.nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_feedback_cb(self, feedback_msg):
        pass  # Could log ETA, distance remaining, etc.

    def _nav_result_cb(self, future):
        self.nav_goal_handle = None
        status = future.result().status

        # action_msgs/GoalStatus constants
        SUCCEEDED = 4
        CANCELED = 5
        ABORTED = 6

        if status == SUCCEEDED:
            self._handle_nav_success()
        elif status == CANCELED:
            self.get_logger().info('Navigation goal was cancelled.')
            # Cancellation is handled by the code that requested it
        else:
            self.get_logger().warn(f'Navigation failed with status {status}.')
            self._handle_nav_failure()

    def _handle_nav_success(self):
        if self.state == State.NAV_TO_PICKUP:
            self.state = State.PICKING_UP
        elif self.state == State.NAV_TO_DELIVERY:
            self.state = State.DROPPING_OFF
        elif self.state == State.NAV_TO_CHARGER:
            self.state = State.CHARGING
            self.get_logger().info('Arrived at charger, waiting for battery.')

    def _handle_nav_failure(self):
        if self.current_order is not None:
            self.get_logger().error(
                f'Navigation failed for order {self.current_order.order_id}. '
                'Marking as FAILED.'
            )
            self.current_order.status = TaskStatus.FAILED
            self.current_order = None
        self.state = State.IDLE

    # =========================================================================
    # Publishers
    # =========================================================================
    def _publish_status(self):
        # Current task
        msg = DeliveryOrderMsg()
        if self.current_order is not None:
            msg.order_id = self.current_order.order_id
            msg.pickup_station = self.current_order.pickup_station
            msg.delivery_station = self.current_order.delivery_station
            msg.priority = self.current_order.priority
            msg.status = self.current_order.status.value
        else:
            msg.order_id = ''
            msg.pickup_station = ''
            msg.delivery_station = ''
            msg.priority = 0
            msg.status = 'idle'
        self.current_task_pub.publish(msg)

        # Task queue
        queue_msg = TaskQueueMsg()
        for _, order in sorted(self.task_queue):
            order_msg = DeliveryOrderMsg()
            order_msg.order_id = order.order_id
            order_msg.pickup_station = order.pickup_station
            order_msg.delivery_station = order.delivery_station
            order_msg.priority = order.priority
            order_msg.status = order.status.value
            queue_msg.orders.append(order_msg)
        self.task_queue_pub.publish(queue_msg)

    # =========================================================================
    # Diagnostics
    # =========================================================================
    def _diag_callback(self, stat):
        stat.summary(DiagnosticStatus.OK, f'State: {self.state}')
        stat.add('state', self.state)
        stat.add('battery_pct', f'{self.battery_pct:.1f}')
        stat.add('queue_length', str(len(self.task_queue)))
        if self.current_order:
            stat.add('current_order_id', self.current_order.order_id)
            stat.add('current_order_status', self.current_order.status.value)
        else:
            stat.add('current_order_id', 'none')
            stat.add('current_order_status', 'idle')

        # Raise warnings / errors
        if self.battery_pct < self.low_battery:
            stat.summary(DiagnosticStatus.WARN, 'Battery low')
        if self.state in (State.NAV_TO_CHARGER, State.CHARGING):
            stat.summary(DiagnosticStatus.WARN, 'Charging')

        return stat


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
