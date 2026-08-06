#ifndef LIDAR_PERCEPTION_SYSTEM__STATIC_OBJ_DETECTOR_HPP_
#define LIDAR_PERCEPTION_SYSTEM__STATIC_OBJ_DETECTOR_HPP_

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

namespace lidar_perception_system
{

struct TargetROI
{
  std::string name;
  bool enabled{true};
  double center_x{0.0};
  double center_y{0.0};
  double roi_size_x{0.6};
  double roi_size_y{0.6};
  double roi_z_min{0.0};
  double roi_z_max{1.0};
};

class StaticObjDetector : public rclcpp::Node
{
public:
  explicit StaticObjDetector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~StaticObjDetector() override = default;

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadParameters();
  void processTarget(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_map,
    const TargetROI & target,
    visualization_msgs::msg::MarkerArray & marker_array,
    int marker_id,
    const rclcpp::Time & stamp);

  // ROS 2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_cloud_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Parameters
  double voxel_leaf_size_{0.05};
  double cluster_tolerance_{0.15};
  int min_cluster_size_{15};
  int max_cluster_size_{5000};
  std::string input_topic_{"/livox/lidar"};
  std::string target_frame_{"map"};

  std::vector<TargetROI> targets_;
};

}  // namespace lidar_perception_system

#endif  // LIDAR_PERCEPTION_SYSTEM__STATIC_OBJ_DETECTOR_HPP_
