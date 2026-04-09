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


def generate_test_description():
    bringup_pkg = get_package_share_directory('warehouse_bringup')

    full_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'full_system.launch.py')
        ),
    )

    return (
        launch.LaunchDescription([
            full_system_launch,
            launch_testing.actions.ReadyToTest(),
        ]),
        {},
    )


class TestDeliveryPipeline(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_delivery_pipeline')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_submit_order_service_available(self):
        """Verify that the /submit_order service is available."""
        from warehouse_interfaces.srv import SubmitOrder
        client = self.node.create_client(SubmitOrder, '/submit_order')

        available = client.wait_for_service(timeout_sec=30.0)
        self.node.destroy_client(client)
        self.assertTrue(available, '/submit_order service not available within 30s')

    def test_cancel_order_service_available(self):
        """Verify that the /cancel_order service is available."""
        from warehouse_interfaces.srv import CancelOrder
        client = self.node.create_client(CancelOrder, '/cancel_order')

        available = client.wait_for_service(timeout_sec=30.0)
        self.node.destroy_client(client)
        self.assertTrue(available, '/cancel_order service not available within 30s')

    def test_submit_order_returns_id(self):
        """Submit an order and verify we get an order ID back."""
        from warehouse_interfaces.srv import SubmitOrder
        client = self.node.create_client(SubmitOrder, '/submit_order')

        available = client.wait_for_service(timeout_sec=30.0)
        if not available:
            self.skipTest('/submit_order service not available')

        request = SubmitOrder.Request()
        request.pickup_station = 'A'
        request.delivery_station = 'D1'
        request.priority = 1

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)

        result = future.result()
        self.assertIsNotNone(result, 'Service call returned no result')
        self.assertTrue(result.accepted, f'Order was not accepted: {result.message}')
        self.assertGreater(len(result.order_id), 0, 'No order_id returned')

        self.node.destroy_client(client)

    def test_task_queue_publishes(self):
        """Verify that /task_queue topic publishes."""
        from warehouse_interfaces.msg import TaskQueue
        msgs = []

        def callback(msg):
            msgs.append(msg)

        sub = self.node.create_subscription(TaskQueue, '/task_queue', callback, 10)

        end_time = self.node.get_clock().now() + rclpy.duration.Duration(seconds=15)
        while self.node.get_clock().now() < end_time and len(msgs) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.5)

        self.node.destroy_subscription(sub)
        self.assertGreater(len(msgs), 0, '/task_queue did not publish within 15s')

    def test_battery_state_publishes(self):
        """Verify that /battery_state topic publishes."""
        from sensor_msgs.msg import BatteryState
        msgs = []

        def callback(msg):
            msgs.append(msg)

        sub = self.node.create_subscription(BatteryState, '/battery_state', callback, 10)

        end_time = self.node.get_clock().now() + rclpy.duration.Duration(seconds=10)
        while self.node.get_clock().now() < end_time and len(msgs) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.5)

        self.node.destroy_subscription(sub)
        self.assertGreater(len(msgs), 0, '/battery_state did not publish within 10s')
        self.assertGreaterEqual(msgs[0].percentage, 0.0)
        self.assertLessEqual(msgs[0].percentage, 100.0)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
