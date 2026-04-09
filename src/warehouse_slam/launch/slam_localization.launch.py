import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('warehouse_slam')
    slam_params = os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml')

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Path to the map file for localization'
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params,
            {'mode': 'localization'},
        ],
        output='screen',
    )

    return LaunchDescription([
        map_file_arg,
        slam_toolbox,
    ])
