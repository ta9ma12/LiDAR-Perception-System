import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_perception_system')
    perception_launch_path = os.path.join(pkg_share, 'launch', 'perception.launch.py')

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception_launch_path),
        launch_arguments={
            'use_dummy_publisher': 'true'
        }.items()
    )

    return LaunchDescription([
        perception_launch,
    ])
