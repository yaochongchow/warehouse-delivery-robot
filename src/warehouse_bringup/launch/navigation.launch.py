from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_pkg = get_package_share_directory('warehouse_bringup')
    nav_pkg = get_package_share_directory('warehouse_navigation')
    nav2_params = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    default_map = os.path.join(nav_pkg, 'maps', 'warehouse_map.yaml')

    # Launch simulation
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'sim.launch.py')
        ),
    )

    # Launch Nav2 navigation after a delay to let Gazebo and robot spawn first
    nav_launch = TimerAction(
        period=30.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_pkg, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={
                    'params_file': nav2_params,
                    'map': default_map,
                    'use_sim_time': 'true',
                }.items(),
            ),
        ],
    )

    return LaunchDescription([
        sim_launch,
        nav_launch,
    ])
