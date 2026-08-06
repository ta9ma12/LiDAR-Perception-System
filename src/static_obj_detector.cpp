#include "lidar_perception_system/static_obj_detector.hpp"

#include <chrono>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace lidar_perception_system
{

StaticObjDetector::StaticObjDetector(const rclcpp::NodeOptions & options)
: Node("static_obj_detector", options)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  loadParameters();

  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&StaticObjDetector::cloudCallback, this, std::placeholders::_1));

  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/debug_markers", 10);
  pub_filtered_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/filtered_cloud", 10);

  RCLCPP_INFO(this->get_logger(), "StaticObjDetector initialized with input: %s, target_frame: %s",
    input_topic_.c_str(), target_frame_.c_str());
}

void StaticObjDetector::loadParameters()
{
  voxel_leaf_size_ = this->declare_parameter<double>("voxel_leaf_size", 0.05);
  cluster_tolerance_ = this->declare_parameter<double>("cluster_tolerance", 0.15);
  min_cluster_size_ = this->declare_parameter<int>("min_cluster_size", 15);
  max_cluster_size_ = this->declare_parameter<int>("max_cluster_size", 5000);
  input_topic_ = this->declare_parameter<std::string>("input_topic", "/livox/lidar");
  target_frame_ = this->declare_parameter<std::string>("target_frame", "map");

  // Load target ROIs
  std::vector<std::string> target_names = {"fixed_bucket_1", "fixed_bucket_2", "fixed_bucket_3", "flag"};

  for (const auto & name : target_names) {
    TargetROI roi;
    roi.name = name;
    roi.enabled = this->declare_parameter<bool>(name + ".enabled", true);
    roi.center_x = this->declare_parameter<double>(name + ".center_x", 0.0);
    roi.center_y = this->declare_parameter<double>(name + ".center_y", 0.0);
    roi.roi_size_x = this->declare_parameter<double>(name + ".roi_size_x", 0.6);
    roi.roi_size_y = this->declare_parameter<double>(name + ".roi_size_y", 0.6);
    roi.roi_z_min = this->declare_parameter<double>(name + ".roi_z_min", 0.0);
    roi.roi_z_max = this->declare_parameter<double>(name + ".roi_z_max", 1.0);

    if (roi.enabled) {
      targets_.push_back(roi);
      RCLCPP_INFO(this->get_logger(), "Loaded Target ROI: %s (Center: [%.2f, %.2f], Z: [%.2f, %.2f])",
        roi.name.c_str(), roi.center_x, roi.center_y, roi.roi_z_min, roi.roi_z_max);
    }
  }
}

void StaticObjDetector::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // 1. Transform cloud to target_frame (e.g. "map")
  sensor_msgs::msg::PointCloud2 cloud_transformed;
  try {
    if (msg->header.frame_id != target_frame_) {
      geometry_msgs::msg::TransformStamped transform_stamped =
        tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp,
          rclcpp::Duration::from_seconds(0.1));
      tf2::doTransform(*msg, cloud_transformed, transform_stamped);
    } else {
      cloud_transformed = *msg;
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Could not transform cloud to %s: %s", target_frame_.c_str(), ex.what());
    return;
  }

  // Convert to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud_transformed, *cloud_raw);

  if (cloud_raw->empty()) {
    return;
  }

  // 2. VoxelGrid downsampling
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ>::Ptr voxel_filter(new pcl::VoxelGrid<pcl::PointXYZ>);
  voxel_filter->setInputCloud(cloud_raw);
  voxel_filter->setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel_filter->filter(*cloud_filtered);

  // Publish filtered cloud for debug
  sensor_msgs::msg::PointCloud2 debug_cloud_msg;
  pcl::toROSMsg(*cloud_filtered, debug_cloud_msg);
  debug_cloud_msg.header.frame_id = target_frame_;
  debug_cloud_msg.header.stamp = msg->header.stamp;
  pub_filtered_cloud_->publish(debug_cloud_msg);

  // 3. Process each Target ROI
  visualization_msgs::msg::MarkerArray marker_array;
  int marker_id = 0;

  for (const auto & target : targets_) {
    processTarget(cloud_filtered, target, marker_array, marker_id++, msg->header.stamp);
  }

  pub_markers_->publish(marker_array);
}

void StaticObjDetector::processTarget(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_map,
  const TargetROI & target,
  visualization_msgs::msg::MarkerArray & marker_array,
  int marker_id,
  const rclcpp::Time & stamp)
{
  // CropBox Filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::CropBox<pcl::PointXYZ> crop_box;
  crop_box.setInputCloud(cloud_map);

  float half_x = static_cast<float>(target.roi_size_x / 2.0);
  float half_y = static_cast<float>(target.roi_size_y / 2.0);

  crop_box.setMin(Eigen::Vector4f(
    static_cast<float>(target.center_x) - half_x,
    static_cast<float>(target.center_y) - half_y,
    static_cast<float>(target.roi_z_min),
    1.0f));
  crop_box.setMax(Eigen::Vector4f(
    static_cast<float>(target.center_x) + half_x,
    static_cast<float>(target.center_y) + half_y,
    static_cast<float>(target.roi_z_max),
    1.0f));
  crop_box.filter(*cropped);

  if (cropped->size() < static_cast<size_t>(min_cluster_size_)) {
    return;
  }

  // Euclidean Clustering
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cropped);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cropped);
  ec.extract(cluster_indices);

  if (cluster_indices.empty()) {
    return;
  }

  // Find largest cluster
  size_t max_size = 0;
  size_t largest_idx = 0;
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (cluster_indices[i].indices.size() > max_size) {
      max_size = cluster_indices[i].indices.size();
      largest_idx = i;
    }
  }

  // Extract largest cluster
  pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster(new pcl::PointCloud<pcl::PointXYZ>);
  for (int idx : cluster_indices[largest_idx].indices) {
    largest_cluster->push_back((*cropped)[idx]);
  }

  // Compute Centroid
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*largest_cluster, centroid);

  // Broadcast TF: target_frame -> fixed_target_<name>
  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped.header.stamp = stamp;
  tf_stamped.header.frame_id = target_frame_;
  tf_stamped.child_frame_id = "fixed_target_" + target.name;
  tf_stamped.transform.translation.x = centroid[0];
  tf_stamped.transform.translation.y = centroid[1];
  tf_stamped.transform.translation.z = centroid[2];
  tf_stamped.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(tf_stamped);

  // Visualization Marker
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = target_frame_;
  marker.ns = "static_targets";
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = centroid[0];
  marker.pose.position.y = centroid[1];
  marker.pose.position.z = centroid[2];
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.8f;
  marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  marker_array.markers.push_back(marker);
}

}  // namespace lidar_perception_system

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar_perception_system::StaticObjDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
