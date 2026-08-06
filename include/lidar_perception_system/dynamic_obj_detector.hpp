#ifndef LIDAR_PERCEPTION_SYSTEM__DYNAMIC_OBJ_DETECTOR_HPP_
#define LIDAR_PERCEPTION_SYSTEM__DYNAMIC_OBJ_DETECTOR_HPP_

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace lidar_perception_system
{

class DynamicObjDetector : public rclcpp::Node
{
public:
  explicit DynamicObjDetector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DynamicObjDetector() override = default;

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadParameters();
  bool loadMapPcd(const std::string & pcd_path);
  pcl::PointCloud<pcl::PointXYZ>::Ptr performBackgroundSubtraction(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud);

  // ROS 2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dynamic_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_cloud_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Buffer for frame accumulation
  std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_buffer_;

  // Map cloud and KD-Tree for background subtraction
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr map_kdtree_;
  bool has_map_{false};

  // Parameters
  double voxel_leaf_size_{0.05};
  double bg_subtraction_radius_{0.10};
  double moving_bucket_z_min_{1.2};
  double moving_bucket_z_max_{2.1};
  int frame_accumulation_count_{3};
  double cluster_tolerance_{0.20};
  int min_cluster_size_{20};
  int max_cluster_size_{10000};
  std::string input_topic_{"/livox/lidar"};
  std::string target_frame_{"map"};
  std::string map_pcd_path_{""};

  // Clear bucket fallback parameters
  bool enable_clear_bucket_fallback_{true};
  double fallback_base_z_min_{0.1};
  double fallback_base_z_max_{1.0};
  double fallback_virtual_z_offset_{0.80};
};

}  // namespace lidar_perception_system

#endif  // LIDAR_PERCEPTION_SYSTEM__DYNAMIC_OBJ_DETECTOR_HPP_
