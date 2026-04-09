import os
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def generate_test_description():
    gazebo_pkg = get_package_share_directory('warehouse_gazebo')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
    )

    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'spawn_robot.launch.py')
        ),
    )

    return (
        launch.LaunchDescription([
            gazebo_launch,
            spawn_launch,
            launch_testing.actions.ReadyToTest(),
        ]),
        {},
    )


class TestRobotSpawns(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_robot_spawns')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_scan_topic_publishes(self):
        """Verify that the lidar publishes /scan messages after robot spawn."""
        msgs_received = []

        def callback(msg):
            msgs_received.append(msg)

        sub = self.node.create_subscription(LaserScan, '/scan', callback, 10)

        # Spin for up to 30 seconds waiting for scan messages
        end_time = self.node.get_clock().now() + rclpy.duration.Duration(seconds=30)
        while self.node.get_clock().now() < end_time and len(msgs_received) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.5)

        self.node.destroy_subscription(sub)
        self.assertGreater(len(msgs_received), 0, '/scan topic did not publish within 30s')

    def test_scan_has_valid_ranges(self):
        """Verify that laser scan has non-empty ranges."""
        msg_received = [None]

        def callback(msg):
            if msg_received[0] is None:
                msg_received[0] = msg

        sub = self.node.create_subscription(LaserScan, '/scan', callback, 10)

        end_time = self.node.get_clock().now() + rclpy.duration.Duration(seconds=30)
        while self.node.get_clock().now() < end_time and msg_received[0] is None:
            rclpy.spin_once(self.node, timeout_sec=0.5)

        self.node.destroy_subscription(sub)
        self.assertIsNotNone(msg_received[0], 'No scan message received')
        self.assertGreater(len(msg_received[0].ranges), 0, 'Scan has no range data')


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check that all processes exited cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
