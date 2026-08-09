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
    default_rviz_path = os.path.join(pkg_share, 'rviz', 'perception_debug.rviz')
    default_map_pcd_path = os.path.join(pkg_share, 'maps', 'robocon2026_field.pcd')

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
        default_value='false',
        description='Whether to launch dummy cloud publisher for testing'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to automatically launch RViz2 with preconfigured settings'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_path,
        description='Path to RViz2 configuration file'
    )

    map_pcd_path_arg = DeclareLaunchArgument(
        'map_pcd_path',
        default_value=default_map_pcd_path,
        description='Path to static map PCD file'
    )

    config_file = LaunchConfiguration('config_file')
    use_dummy = LaunchConfiguration('use_dummy_publisher')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    localization_type = LaunchConfiguration('localization_type')
    map_pcd_path = LaunchConfiguration('map_pcd_path')

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
        parameters=[config_file, {'map_pcd_path': map_pcd_path}]
    )

    dummy_publisher_node = Node(
        package='lidar_perception_system',
        executable='dummy_cloud_publisher',
        name='dummy_cloud_publisher',
        output='screen',
        condition=IfCondition(use_dummy)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    log_localization = LogInfo(
        msg=['Selected Localization Type: ', localization_type]
    )

    return LaunchDescription([
        config_file_arg,
        localization_type_arg,
        use_dummy_arg,
        use_rviz_arg,
        rviz_config_arg,
        map_pcd_path_arg,
        log_localization,
        static_detector_node,
        dynamic_detector_node,
        dummy_publisher_node,
        rviz_node,
    ])
