import os
import unittest
import math

import launch
import launch_testing
import launch_testing.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus


def generate_test_description():
    bringup_pkg = get_package_share_directory('warehouse_bringup')

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'navigation.launch.py')
        ),
    )

    return (
        launch.LaunchDescription([
            nav_launch,
            launch_testing.actions.ReadyToTest(),
        ]),
        {},
    )


class TestNavigation(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_navigation')
        cls.nav_client = ActionClient(cls.node, NavigateToPose, 'navigate_to_pose')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_nav2_action_server_available(self):
        """Verify that the Nav2 NavigateToPose action server is available."""
        available = self.nav_client.wait_for_server(timeout_sec=60.0)
        self.assertTrue(available, 'NavigateToPose action server not available within 60s')

    def test_navigate_to_goal(self):
        """Send a navigation goal and verify the robot reaches it."""
        available = self.nav_client.wait_for_server(timeout_sec=60.0)
        if not available:
            self.skipTest('Nav2 action server not available')

        # Send goal to a nearby open area
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = 0.0
        goal_msg.pose.pose.position.y = -5.0
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        goal_handle = future.result()

        self.assertIsNotNone(goal_handle, 'Goal was not accepted')
        self.assertTrue(goal_handle.accepted, 'Navigation goal was rejected')

        # Wait for result (up to 120 seconds for navigation)
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=120.0)

        result = result_future.result()
        self.assertIsNotNone(result, 'Navigation did not complete within 120s')
        self.assertEqual(
            result.status,
            GoalStatus.STATUS_SUCCEEDED,
            f'Navigation failed with status: {result.status}',
        )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
