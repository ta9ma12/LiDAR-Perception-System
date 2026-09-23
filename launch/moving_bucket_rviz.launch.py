"""Start the CUDA detector together with its RViz2 inspection layout."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lidar_perception_system')
    detector_launch = os.path.join(share, 'launch', 'moving_bucket.launch.py')
    rviz_config = os.path.join(share, 'rviz', 'moving_bucket.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=os.path.join(
            share, 'config', 'moving_bucket.json')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(detector_launch),
            launch_arguments={
                'config_file': LaunchConfiguration('config_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
