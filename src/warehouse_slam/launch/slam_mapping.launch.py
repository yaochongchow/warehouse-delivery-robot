import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('warehouse_slam')
    slam_params = os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml')

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params],
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('warehouse_robot_description'),
            'rviz', 'robot.rviz'
        )],
        output='screen',
    )

    return LaunchDescription([
        slam_toolbox,
        rviz2,
    ])
