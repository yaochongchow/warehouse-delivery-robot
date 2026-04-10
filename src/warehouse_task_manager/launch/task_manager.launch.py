import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_task_manager')
    stations_yaml = os.path.join(pkg_share, 'config', 'stations.yaml')

    return LaunchDescription([
        Node(
            package='warehouse_task_manager',
            executable='task_manager_node',
            name='task_manager_node',
            output='screen',
            parameters=[{
                'stations_yaml': stations_yaml,
                'low_battery_threshold': 20.0,
                'charge_resume_threshold': 80.0,
                'pickup_duration': 2.0,
                'dropoff_duration': 2.0,
                'use_manipulation_actions': True,
            }],
        ),
        Node(
            package='warehouse_task_manager',
            executable='battery_simulator',
            name='battery_simulator',
            output='screen',
            parameters=[{
                'stations_yaml': stations_yaml,
                'initial_level': 100.0,
                'drain_rate_per_min': 0.5,
                'charge_rate_per_min': 5.0,
                'charge_radius': 1.0,
            }],
        ),
        Node(
            package='warehouse_task_manager',
            executable='manipulation_simulator',
            name='manipulation_simulator',
            output='screen',
            parameters=[{
                'pick_duration': 2.0,
                'place_duration': 2.0,
                'failure_rate': 0.0,
            }],
        ),
    ])
