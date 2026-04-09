import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_dashboard')
    web_dir = os.path.join(pkg_share, 'web')

    return LaunchDescription([
        # rosbridge_websocket for roslibjs communication
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': 9090,
                'address': '',
                'retry_startup_delay': 5.0,
            }],
        ),

        # Simple HTTP server to serve web dashboard files
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8080', '--directory', web_dir],
            name='web_server',
            output='screen',
        ),

        # Rosbag fault recorder runs alongside
        Node(
            package='warehouse_dashboard',
            executable='rosbag_fault_recorder',
            name='rosbag_fault_recorder',
            output='screen',
        ),
    ])
