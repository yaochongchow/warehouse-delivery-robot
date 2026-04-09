"""
Rosbag Fault Recorder -- Triggers rosbag recording on diagnostic errors.

Subscribes to /diagnostics (diagnostic_msgs/DiagnosticArray).  When any
status is ERROR (level=2) or STALE (level=3), triggers a 30-second
rosbag recording via subprocess.  Cooldown: minimum 60 seconds between
recordings.  Bags are saved to ~/rosbags/fault_{timestamp}/.
"""

import os
import subprocess
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


class RosbagFaultRecorderNode(Node):
    def __init__(self):
        super().__init__('rosbag_fault_recorder')

        # ---- Parameters --------------------------------------------------------
        self.declare_parameter('record_duration', 30.0)
        self.declare_parameter('cooldown', 60.0)
        self.declare_parameter('output_dir', str(Path.home() / 'rosbags'))
        self.declare_parameter('topics', [
            '/current_task',
            '/task_queue',
            '/battery_state',
            '/diagnostics',
            '/odom',
            '/scan',
            '/tf',
            '/tf_static',
        ])

        self.record_duration = self.get_parameter('record_duration').value
        self.cooldown = self.get_parameter('cooldown').value
        self.output_dir = self.get_parameter('output_dir').value
        self.topics = self.get_parameter('topics').value

        # ---- State -------------------------------------------------------------
        self.last_record_time = 0.0
        self.recording_process: subprocess.Popen | None = None

        # ---- Ensure output directory exists ------------------------------------
        os.makedirs(self.output_dir, exist_ok=True)

        # ---- Subscriber --------------------------------------------------------
        self.create_subscription(
            DiagnosticArray, '/diagnostics', self._diag_cb, 10)

        # ---- Timer to check recording completion ------------------------------
        self.create_timer(2.0, self._check_recording)

        self.get_logger().info(
            f'Rosbag Fault Recorder started. Output: {self.output_dir}'
        )

    def _diag_cb(self, msg: DiagnosticArray):
        fault_detected = False
        fault_details = []

        for status in msg.status:
            if status.level in (DiagnosticStatus.ERROR, DiagnosticStatus.STALE):
                fault_detected = True
                level_name = 'ERROR' if status.level == DiagnosticStatus.ERROR else 'STALE'
                fault_details.append(f'{level_name}: {status.name} - {status.message}')

        if not fault_detected:
            return

        now = time.time()
        if now - self.last_record_time < self.cooldown:
            remaining = self.cooldown - (now - self.last_record_time)
            self.get_logger().info(
                f'Fault detected but in cooldown ({remaining:.0f}s remaining).'
            )
            return

        if self.recording_process is not None and self.recording_process.poll() is None:
            self.get_logger().info('Fault detected but recording already in progress.')
            return

        # Trigger recording
        for detail in fault_details:
            self.get_logger().warn(f'Fault: {detail}')

        self._start_recording()

    def _start_recording(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_dir = os.path.join(self.output_dir, f'fault_{timestamp}')

        cmd = [
            'ros2', 'bag', 'record',
            '--output', bag_dir,
            '--max-bag-duration', str(int(self.record_duration)),
        ] + list(self.topics)

        self.get_logger().info(
            f'Starting {self.record_duration}s rosbag recording to {bag_dir}'
        )

        try:
            self.recording_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            self.last_record_time = time.time()

            # Schedule termination after record_duration seconds
            self.create_timer(
                self.record_duration,
                self._stop_recording,
            )
        except Exception as e:
            self.get_logger().error(f'Failed to start rosbag recording: {e}')

    def _stop_recording(self):
        if self.recording_process is not None and self.recording_process.poll() is None:
            self.get_logger().info('Stopping rosbag recording.')
            self.recording_process.terminate()
            try:
                self.recording_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.recording_process.kill()
            self.recording_process = None

    def _check_recording(self):
        if self.recording_process is not None and self.recording_process.poll() is not None:
            rc = self.recording_process.returncode
            if rc != 0 and rc != -15:  # -15 = SIGTERM (normal stop)
                stderr = self.recording_process.stderr.read().decode() if self.recording_process.stderr else ''
                self.get_logger().warn(
                    f'Rosbag recording exited with code {rc}: {stderr[:200]}'
                )
            self.recording_process = None


def main(args=None):
    rclpy.init(args=args)
    node = RosbagFaultRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_recording()  # Clean up any active recording
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
