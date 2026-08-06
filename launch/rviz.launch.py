import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_perception_system')
    default_rviz_path = os.path.join(pkg_share, 'rviz', 'perception_debug.rviz')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_path,
        description='Path to RViz2 configuration file'
    )

    rviz_config = LaunchConfiguration('rviz_config')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        rviz_config_arg,
        rviz_node,
    ])
