import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('warehouse_gazebo')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(pkg_dir, 'worlds', 'warehouse.world')
    models_dir = os.path.join(pkg_dir, 'models')

    # Set GAZEBO_MODEL_PATH so Gazebo can find custom models
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_dir,
    )

    # Set GAZEBO_PLUGIN_PATH so Gazebo can find ROS2 plugins
    set_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value='/opt/ros/humble/lib',
    )

    # Launch Gazebo server + client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
            'pause': 'false',
            'gui': 'false',
        }.items(),
    )

    return LaunchDescription([
        set_model_path,
        set_plugin_path,
        gazebo,
    ])
