import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('warehouse_navigation')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock',
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to the map yaml file',
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params,
        description='Full path to the Nav2 parameters file',
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml_file},
        ],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params_file,
        map_server,
        amcl,
        lifecycle_manager_localization,
    ])
