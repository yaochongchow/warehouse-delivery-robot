from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_pkg = get_package_share_directory('warehouse_bringup')
    slam_pkg = get_package_share_directory('warehouse_slam')

    # Launch simulation
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'sim.launch.py')
        ),
    )

    # Launch SLAM mapping
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'slam_mapping.launch.py')
        ),
    )

    return LaunchDescription([
        sim_launch,
        slam_launch,
    ])
