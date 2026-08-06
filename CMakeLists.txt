cmake_minimum_required(VERSION 3.8)
project(lidar_perception_system)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# Find PCL
find_package(PCL REQUIRED COMPONENTS common io filters segmentation search kdtree)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# Static Object Detector node
add_executable(static_obj_detector src/static_obj_detector.cpp)
ament_target_dependencies(static_obj_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
)
target_link_libraries(static_obj_detector ${PCL_LIBRARIES})

# Dynamic Object Detector node
add_executable(dynamic_obj_detector src/dynamic_obj_detector.cpp)
ament_target_dependencies(dynamic_obj_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
)
target_link_libraries(dynamic_obj_detector ${PCL_LIBRARIES})

# Dummy Cloud Publisher for testing
add_executable(dummy_cloud_publisher src/dummy_cloud_publisher.cpp)
ament_target_dependencies(dummy_cloud_publisher
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  pcl_conversions
  pcl_ros
)
target_link_libraries(dummy_cloud_publisher ${PCL_LIBRARIES})

# Install executables
install(TARGETS
  static_obj_detector
  dynamic_obj_detector
  dummy_cloud_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install include headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch, config, maps, and rviz directories
install(DIRECTORY launch config maps rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
