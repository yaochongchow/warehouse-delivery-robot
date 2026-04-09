import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_dashboard')
    params_file = os.path.join(pkg_share, 'config', 'dashboard_params.yaml')

    return LaunchDescription([
        Node(
            package='warehouse_dashboard',
            executable='terminal_dashboard',
            name='terminal_dashboard',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='warehouse_dashboard',
            executable='rosbag_fault_recorder',
            name='rosbag_fault_recorder',
            output='screen',
            parameters=[params_file],
        ),
    ])
