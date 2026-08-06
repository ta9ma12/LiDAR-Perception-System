import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_perception_system')
    default_config_path = os.path.join(pkg_share, 'config', 'perception_params.yaml')

    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to the perception parameters YAML file'
    )

    localization_type_arg = DeclareLaunchArgument(
        'localization_type',
        default_value='glim',
        description='Localization node type to use: "glim" or "ndt"'
    )

    use_dummy_arg = DeclareLaunchArgument(
        'use_dummy_publisher',
        default_value='true',
        description='Whether to launch dummy cloud publisher for testing'
    )

    config_file = LaunchConfiguration('config_file')
    use_dummy = LaunchConfiguration('use_dummy_publisher')
    localization_type = LaunchConfiguration('localization_type')

    # Nodes
    static_detector_node = Node(
        package='lidar_perception_system',
        executable='static_obj_detector',
        name='static_obj_detector',
        output='screen',
        parameters=[config_file]
    )

    dynamic_detector_node = Node(
        package='lidar_perception_system',
        executable='dynamic_obj_detector',
        name='dynamic_obj_detector',
        output='screen',
        parameters=[config_file]
    )

    dummy_publisher_node = Node(
        package='lidar_perception_system',
        executable='dummy_cloud_publisher',
        name='dummy_cloud_publisher',
        output='screen',
        condition=IfCondition(use_dummy)
    )

    log_localization = LogInfo(
        msg=['Selected Localization Type: ', localization_type]
    )

    return LaunchDescription([
        config_file_arg,
        localization_type_arg,
        use_dummy_arg,
        log_localization,
        static_detector_node,
        dynamic_detector_node,
        dummy_publisher_node,
    ])
