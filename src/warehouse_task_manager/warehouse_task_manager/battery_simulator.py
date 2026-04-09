"""
Battery Simulator Node

Publishes sensor_msgs/BatteryState at 1 Hz.
- Starts at 100 % charge.
- Drains 0.5 %/min when the robot is moving (|velocity| > 0.01 m/s on /odom).
- Charges 5 %/min when the robot is within 1.0 m of the configured charging
  station pose.
- Exposes a 'battery_level' parameter that can be set at runtime to override
  the current level (useful for testing low-battery scenarios).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry

from ament_index_python.packages import get_package_share_directory
from warehouse_task_manager.stations import load_stations


class BatterySimulatorNode(Node):
    def __init__(self):
        super().__init__('battery_simulator')

        # ---- Parameters --------------------------------------------------------
        self.declare_parameter('initial_level', 100.0)
        self.declare_parameter('drain_rate_per_min', 0.5)
        self.declare_parameter('charge_rate_per_min', 5.0)
        self.declare_parameter('charge_radius', 1.0)
        self.declare_parameter('stations_yaml', '')
        # Mutable override -- set via `ros2 param set`
        self.declare_parameter('battery_level', -1.0)

        self.level = self.get_parameter('initial_level').value          # percent
        self.drain_rate = self.get_parameter('drain_rate_per_min').value  # %/min
        self.charge_rate = self.get_parameter('charge_rate_per_min').value
        self.charge_radius = self.get_parameter('charge_radius').value

        # ---- Load charging station pose ----------------------------------------
        stations_yaml = self.get_parameter('stations_yaml').get_parameter_value().string_value
        if not stations_yaml:
            pkg_share = get_package_share_directory('warehouse_task_manager')
            stations_yaml = pkg_share + '/config/stations.yaml'

        stations = load_stations(stations_yaml)
        if 'charging' in stations:
            p = stations['charging'].pose.position
            self.charge_x = p.x
            self.charge_y = p.y
            self.get_logger().info(
                f'Charging station at ({self.charge_x:.1f}, {self.charge_y:.1f})'
            )
        else:
            self.charge_x = None
            self.charge_y = None
            self.get_logger().warn('No charging station found in stations config.')

        # ---- Odom state --------------------------------------------------------
        self.robot_speed = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0

        # ---- Subscribers -------------------------------------------------------
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # ---- Publisher ---------------------------------------------------------
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)

        # ---- Parameter callback ------------------------------------------------
        self.add_on_set_parameters_callback(self._param_cb)

        # ---- Timer (1 Hz) ------------------------------------------------------
        self.create_timer(1.0, self._tick)

        self.get_logger().info('Battery Simulator started.')

    # =========================================================================
    def _param_cb(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'battery_level' and p.value >= 0.0:
                self.level = min(max(p.value, 0.0), 100.0)
                self.get_logger().info(f'Battery level overridden to {self.level:.1f}%')
        return SetParametersResult(successful=True)

    def _odom_cb(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.robot_speed = math.sqrt(vx * vx + vy * vy)
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def _near_charger(self) -> bool:
        if self.charge_x is None:
            return False
        dx = self.robot_x - self.charge_x
        dy = self.robot_y - self.charge_y
        return math.sqrt(dx * dx + dy * dy) <= self.charge_radius

    def _tick(self):
        # Check for runtime override
        override = self.get_parameter('battery_level').value
        if override >= 0.0:
            self.level = min(max(override, 0.0), 100.0)

        dt_min = 1.0 / 60.0  # 1 second expressed in minutes

        if self._near_charger():
            self.level = min(100.0, self.level + self.charge_rate * dt_min)
        elif self.robot_speed > 0.01:
            self.level = max(0.0, self.level - self.drain_rate * dt_min)

        # Build and publish message
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.voltage = 12.0 * (self.level / 100.0)       # rough model
        msg.current = -0.5 if self.robot_speed > 0.01 else 0.0
        msg.charge = float('nan')
        msg.capacity = float('nan')
        msg.design_capacity = float('nan')
        msg.percentage = self.level / 100.0              # 0.0 - 1.0
        msg.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING
            if self._near_charger()
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True

        self.battery_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatterySimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
