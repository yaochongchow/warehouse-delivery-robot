"""
Manipulation Simulator Node.

Provides virtual pick/place action servers so the delivery pipeline can run:
  Navigate -> PickObject -> Navigate -> PlaceObject

This node keeps a tiny in-memory shelf/object model for demo purposes.
"""

import random
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from warehouse_interfaces.action import PickObject, PlaceObject


class ManipulationSimulator(Node):
    def __init__(self):
        super().__init__('manipulation_simulator')

        self.declare_parameter('pick_duration', 2.0)
        self.declare_parameter('place_duration', 2.0)
        self.declare_parameter('failure_rate', 0.0)

        self.pick_duration = float(self.get_parameter('pick_duration').value)
        self.place_duration = float(self.get_parameter('place_duration').value)
        self.failure_rate = float(self.get_parameter('failure_rate').value)

        # Very small world state for simulation/demo.
        self.station_objects = {
            'pickup_A': 'box_A',
            'pickup_B': 'box_B',
        }
        self.held_objects = {}  # order_id -> object_id

        self.pick_server = ActionServer(
            self,
            PickObject,
            '/pick_object',
            execute_callback=self._execute_pick,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.place_server = ActionServer(
            self,
            PlaceObject,
            '/place_object',
            execute_callback=self._execute_place,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self.get_logger().info('Manipulation Simulator started.')

    def _goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _run_progress(self, goal_handle, duration, phase_prefix, feedback_type):
        start = time.time()
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False, 'Canceled by client.'

            elapsed = time.time() - start
            progress = min(1.0, elapsed / max(duration, 0.001))

            feedback = feedback_type()
            feedback.phase = phase_prefix
            feedback.progress = float(progress)
            goal_handle.publish_feedback(feedback)

            if progress >= 1.0:
                return True, 'Completed.'

            time.sleep(0.1)

    def _execute_pick(self, goal_handle):
        req = goal_handle.request
        result = PickObject.Result()

        station = req.station_name
        requested_object = req.object_id.strip()
        available_object = self.station_objects.get(station, '')

        if not available_object:
            goal_handle.abort()
            result.success = False
            result.message = f'No object available at station "{station}".'
            result.picked_object_id = ''
            return result

        if requested_object and requested_object != available_object:
            goal_handle.abort()
            result.success = False
            result.message = (
                f'Requested object "{requested_object}" not present at "{station}".'
            )
            result.picked_object_id = ''
            return result

        ok, msg = self._run_progress(
            goal_handle, self.pick_duration, 'grasping', PickObject.Feedback
        )
        if not ok:
            result.success = False
            result.message = msg
            result.picked_object_id = ''
            return result

        if random.random() < self.failure_rate:
            goal_handle.abort()
            result.success = False
            result.message = 'Simulated pick failure.'
            result.picked_object_id = ''
            return result

        picked = self.station_objects.pop(station)
        self.held_objects[req.order_id] = picked

        goal_handle.succeed()
        result.success = True
        result.message = f'Picked "{picked}" from "{station}".'
        result.picked_object_id = picked
        self.get_logger().info(result.message)
        return result

    def _execute_place(self, goal_handle):
        req = goal_handle.request
        result = PlaceObject.Result()

        if req.order_id not in self.held_objects:
            goal_handle.abort()
            result.success = False
            result.message = f'No held object for order "{req.order_id}".'
            result.placed_object_id = ''
            return result

        ok, msg = self._run_progress(
            goal_handle, self.place_duration, 'placing', PlaceObject.Feedback
        )
        if not ok:
            result.success = False
            result.message = msg
            result.placed_object_id = ''
            return result

        if random.random() < self.failure_rate:
            goal_handle.abort()
            result.success = False
            result.message = 'Simulated place failure.'
            result.placed_object_id = ''
            return result

        placed = self.held_objects.pop(req.order_id)
        self.station_objects[req.station_name] = placed

        goal_handle.succeed()
        result.success = True
        result.message = f'Placed "{placed}" at "{req.station_name}".'
        result.placed_object_id = placed
        self.get_logger().info(result.message)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
