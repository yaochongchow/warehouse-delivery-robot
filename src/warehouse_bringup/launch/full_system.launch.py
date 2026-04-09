from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_pkg = get_package_share_directory('warehouse_bringup')
    task_mgr_pkg = get_package_share_directory('warehouse_task_manager')
    dashboard_pkg = get_package_share_directory('warehouse_dashboard')

    # Launch simulation + navigation
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'navigation.launch.py')
        ),
    )

    # Launch task manager
    task_mgr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(task_mgr_pkg, 'launch', 'task_manager.launch.py')
        ),
    )

    # Launch terminal dashboard
    dashboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dashboard_pkg, 'launch', 'terminal_dashboard.launch.py')
        ),
    )

    return LaunchDescription([
        nav_launch,
        task_mgr_launch,
        dashboard_launch,
    ])
