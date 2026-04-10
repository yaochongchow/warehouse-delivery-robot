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
from rclpy.time import Time
from rclpy.duration import Duration

from ament_index_python.packages import get_package_share_directory

from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

from warehouse_interfaces.msg import DeliveryOrder as DeliveryOrderMsg
from warehouse_interfaces.msg import TaskQueue as TaskQueueMsg
from warehouse_interfaces.srv import SubmitOrder, CancelOrder
from warehouse_interfaces.action import PickObject, PlaceObject

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
        self.declare_parameter('use_manipulation_actions', True)

        stations_yaml = self.get_parameter('stations_yaml').get_parameter_value().string_value
        if not stations_yaml:
            pkg_share = get_package_share_directory('warehouse_task_manager')
            stations_yaml = pkg_share + '/config/stations.yaml'

        self.low_battery = self.get_parameter('low_battery_threshold').value
        self.charge_resume = self.get_parameter('charge_resume_threshold').value
        self.pickup_duration = self.get_parameter('pickup_duration').value
        self.dropoff_duration = self.get_parameter('dropoff_duration').value
        self.use_manipulation_actions = bool(
            self.get_parameter('use_manipulation_actions').value
        )

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
        self.pick_goal_handle = None
        self.place_goal_handle = None
        self.manipulation_in_progress = False
        self.wait_start: float | None = None
        self.interrupted_order: DeliveryOrder | None = None
        self._nav_unready_logged = False
        self._nav_unready_reason = ''
        self.next_dispatch_time: float = 0.0

        # ---- Callback group for concurrent service / action handling -----------
        self.cb_group = ReentrantCallbackGroup()

        # ---- Nav2 action client ------------------------------------------------
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.cb_group,
        )
        self.pick_client = ActionClient(
            self, PickObject, '/pick_object', callback_group=self.cb_group
        )
        self.place_client = ActionClient(
            self, PlaceObject, '/place_object', callback_group=self.cb_group
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

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
            self._cancel_active_goals()
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
                self._cancel_active_goals()
            self.state = State.NAV_TO_CHARGER
            if not self._send_nav_goal('charging'):
                self.state = State.IDLE
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
            if time.time() < self.next_dispatch_time:
                return
            # Avoid consuming orders until Nav2 and required TF links are ready.
            nav_ready, reason = self._is_navigation_ready()
            if not nav_ready:
                if not self._nav_unready_logged:
                    self.get_logger().warn(
                        f'Navigation not ready yet ({reason}); keeping orders queued.'
                    )
                    self._nav_unready_reason = reason
                    self._nav_unready_logged = True
                self.next_dispatch_time = time.time() + 1.0
                return
            self._nav_unready_logged = False
            self._nav_unready_reason = ''
            _, order = heapq.heappop(self.task_queue)
            self.current_order = order
            self.current_order.status = TaskStatus.NAVIGATING_TO_PICKUP
            self.state = State.NAV_TO_PICKUP
            self.get_logger().info(
                f'Starting order {order.order_id}: navigating to {order.pickup_station}'
            )
            if not self._send_nav_goal(order.pickup_station):
                # Re-queue if Nav2 dropped out between readiness check and send.
                self.current_order.status = TaskStatus.PENDING
                heapq.heappush(self.task_queue, (self.current_order.priority, self.current_order))
                self.current_order = None
                self.state = State.IDLE
                self.next_dispatch_time = time.time() + 1.0

        # ---------- PICKING_UP / DROPPING_OFF (timed wait) -------------------
        elif self.state == State.PICKING_UP:
            if self.manipulation_in_progress:
                return
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
                if not self._send_nav_goal(self.current_order.delivery_station):
                    self._handle_nav_failure()

        elif self.state == State.DROPPING_OFF:
            if self.manipulation_in_progress:
                return
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
    def _cancel_active_goals(self):
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None
        if self.pick_goal_handle is not None:
            self.pick_goal_handle.cancel_goal_async()
            self.pick_goal_handle = None
        if self.place_goal_handle is not None:
            self.place_goal_handle.cancel_goal_async()
            self.place_goal_handle = None
        self.manipulation_in_progress = False
        self.wait_start = None

    def _send_nav_goal(self, station_name: str) -> bool:
        if station_name not in self.stations:
            self.get_logger().error(f'Station "{station_name}" not found in stations map.')
            return False
        nav_ready, reason = self._is_navigation_ready()
        if not nav_ready:
            self.get_logger().warn(f'Navigation unavailable ({reason}); delaying goal send.')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        src = self.stations[station_name]
        goal_pose.header.frame_id = src.header.frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = src.pose
        goal_msg.pose = goal_pose

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 action server not available yet.')
            return False

        self.get_logger().info(f'Sending navigation goal to station "{station_name}"')
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self._nav_feedback_cb
        )
        send_goal_future.add_done_callback(self._nav_goal_response_cb)
        return True

    def _is_navigation_ready(self) -> tuple[bool, str]:
        if not self.nav_client.server_is_ready():
            return False, 'navigate_to_pose action server unavailable'

        required_tf_links = (
            ('odom', 'base_link'),
            ('map', 'odom'),
            ('map', 'base_link'),
        )
        for target_frame, source_frame in required_tf_links:
            if not self.tf_buffer.can_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            ):
                return False, f'missing TF: {source_frame} -> {target_frame}'

        return True, 'ready'

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal was rejected; will retry later.')
            if self.current_order is not None and self.state in (State.NAV_TO_PICKUP, State.NAV_TO_DELIVERY):
                self.current_order.status = TaskStatus.PENDING
                heapq.heappush(self.task_queue, (self.current_order.priority, self.current_order))
                self.current_order = None
            self.state = State.IDLE
            self.next_dispatch_time = time.time() + 2.0
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
            if self.state in (State.NAV_TO_PICKUP, State.NAV_TO_DELIVERY):
                # Nav2 lifecycle resets can cancel in-flight goals. Re-queue
                # the order so it retries after Nav2 returns active.
                if self.current_order is not None:
                    self.current_order.status = TaskStatus.PENDING
                    heapq.heappush(
                        self.task_queue, (self.current_order.priority, self.current_order)
                    )
                    self.current_order = None
                self.state = State.IDLE
                self.next_dispatch_time = time.time() + 2.0
            elif self.state == State.NAV_TO_CHARGER:
                # Retry charging navigation on next tick if it was canceled.
                self.state = State.IDLE
                self.next_dispatch_time = time.time() + 1.0
            # Other cancellations are intentionally no-op.
        else:
            self.get_logger().warn(f'Navigation failed with status {status}.')
            self._handle_nav_failure()

    def _handle_nav_success(self):
        if self.state == State.NAV_TO_PICKUP:
            if self.use_manipulation_actions and self._start_pick_action():
                return
            self.state = State.PICKING_UP
            self.wait_start = None
            self.get_logger().warn(
                'Pick action unavailable; using timed pickup fallback.'
            )
        elif self.state == State.NAV_TO_DELIVERY:
            if self.use_manipulation_actions and self._start_place_action():
                return
            self.state = State.DROPPING_OFF
            self.wait_start = None
            self.get_logger().warn(
                'Place action unavailable; using timed dropoff fallback.'
            )
        elif self.state == State.NAV_TO_CHARGER:
            self.state = State.CHARGING
            self.get_logger().info('Arrived at charger, waiting for battery.')

    # =========================================================================
    # Manipulation helpers
    # =========================================================================
    def _start_pick_action(self) -> bool:
        if self.current_order is None:
            return False
        if not self.pick_client.wait_for_server(timeout_sec=1.0):
            return False

        goal_msg = PickObject.Goal()
        goal_msg.order_id = self.current_order.order_id
        goal_msg.station_name = self.current_order.pickup_station
        goal_msg.object_id = ''

        self.state = State.PICKING_UP
        self.current_order.status = TaskStatus.PICKING_UP
        self.wait_start = None
        self.manipulation_in_progress = True
        self.get_logger().info(
            f'Sending pick action at "{self.current_order.pickup_station}"'
        )
        send_goal_future = self.pick_client.send_goal_async(
            goal_msg, feedback_callback=self._pick_feedback_cb
        )
        send_goal_future.add_done_callback(self._pick_goal_response_cb)
        return True

    def _pick_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.manipulation_in_progress = False
            self.get_logger().warn('Pick action rejected; using timed pickup fallback.')
            if self.state == State.PICKING_UP:
                self.wait_start = None
            return

        self.pick_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._pick_result_cb)

    def _pick_feedback_cb(self, feedback_msg):
        pass

    def _pick_result_cb(self, future):
        self.pick_goal_handle = None
        self.manipulation_in_progress = False
        if self.current_order is None or self.state != State.PICKING_UP:
            return

        status = future.result().status
        result = future.result().result
        SUCCEEDED = 4

        if status == SUCCEEDED and result.success:
            self.current_order.status = TaskStatus.NAVIGATING_TO_DELIVERY
            self.state = State.NAV_TO_DELIVERY
            self.get_logger().info(
                f'Pick complete, navigating to {self.current_order.delivery_station}'
            )
            if not self._send_nav_goal(self.current_order.delivery_station):
                self._handle_nav_failure()
            return

        self.get_logger().warn(f'Pick failed: {result.message}')
        self._handle_manipulation_failure()

    def _start_place_action(self) -> bool:
        if self.current_order is None:
            return False
        if not self.place_client.wait_for_server(timeout_sec=1.0):
            return False

        goal_msg = PlaceObject.Goal()
        goal_msg.order_id = self.current_order.order_id
        goal_msg.station_name = self.current_order.delivery_station

        self.state = State.DROPPING_OFF
        self.current_order.status = TaskStatus.DROPPING_OFF
        self.wait_start = None
        self.manipulation_in_progress = True
        self.get_logger().info(
            f'Sending place action at "{self.current_order.delivery_station}"'
        )
        send_goal_future = self.place_client.send_goal_async(
            goal_msg, feedback_callback=self._place_feedback_cb
        )
        send_goal_future.add_done_callback(self._place_goal_response_cb)
        return True

    def _place_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.manipulation_in_progress = False
            self.get_logger().warn('Place action rejected; using timed dropoff fallback.')
            if self.state == State.DROPPING_OFF:
                self.wait_start = None
            return

        self.place_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._place_result_cb)

    def _place_feedback_cb(self, feedback_msg):
        pass

    def _place_result_cb(self, future):
        self.place_goal_handle = None
        self.manipulation_in_progress = False
        if self.current_order is None or self.state != State.DROPPING_OFF:
            return

        status = future.result().status
        result = future.result().result
        SUCCEEDED = 4

        if status == SUCCEEDED and result.success:
            self.current_order.status = TaskStatus.COMPLETED
            self.get_logger().info(
                f'Order {self.current_order.order_id} completed!'
            )
            self.current_order = None
            self.state = State.IDLE
            return

        self.get_logger().warn(f'Place failed: {result.message}')
        self._handle_manipulation_failure()

    def _handle_manipulation_failure(self):
        if self.current_order is not None:
            self.current_order.status = TaskStatus.PENDING
            heapq.heappush(self.task_queue, (self.current_order.priority, self.current_order))
            self.get_logger().warn(
                f'Manipulation failed for order {self.current_order.order_id}; '
                're-queueing to retry.'
            )
            self.current_order = None
        self.state = State.IDLE
        self.next_dispatch_time = time.time() + 2.0

    def _handle_nav_failure(self):
        if self.current_order is not None:
            # Navigation can fail transiently while Nav2 recovers. Re-queue
            # instead of permanently failing the order.
            self.get_logger().warn(
                f'Navigation failed for order {self.current_order.order_id}; '
                're-queueing to retry.'
            )
            self.current_order.status = TaskStatus.PENDING
            heapq.heappush(self.task_queue, (self.current_order.priority, self.current_order))
            self.current_order = None
        self.state = State.IDLE
        self.next_dispatch_time = time.time() + 2.0

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
