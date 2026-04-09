from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_pkg = get_package_share_directory('warehouse_bringup')

    # Launch full system
    full_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'full_system.launch.py')
        ),
    )

    # Submit demo orders after a delay to let the system initialize
    submit_order_1 = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call', '/submit_order',
                    'warehouse_interfaces/srv/SubmitOrder',
                    '{pickup_station: "A", delivery_station: "D1", priority: 1}',
                ],
                output='screen',
            ),
        ],
    )

    submit_order_2 = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call', '/submit_order',
                    'warehouse_interfaces/srv/SubmitOrder',
                    '{pickup_station: "B", delivery_station: "D2", priority: 2}',
                ],
                output='screen',
            ),
        ],
    )

    submit_order_3 = TimerAction(
        period=25.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call', '/submit_order',
                    'warehouse_interfaces/srv/SubmitOrder',
                    '{pickup_station: "A", delivery_station: "D2", priority: 3}',
                ],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        full_system_launch,
        submit_order_1,
        submit_order_2,
        submit_order_3,
    ])
