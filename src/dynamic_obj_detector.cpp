#include "lidar_perception_system/dynamic_obj_detector.hpp"

#include <chrono>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace lidar_perception_system
{

DynamicObjDetector::DynamicObjDetector(const rclcpp::NodeOptions & options)
: Node("dynamic_obj_detector", options),
  map_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
  map_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  loadParameters();

  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&DynamicObjDetector::cloudCallback, this, std::placeholders::_1));

  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/debug_markers", 10);
  pub_dynamic_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/dynamic_cloud", 10);
  pub_map_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/map_cloud", rclcpp::QoS(1).transient_local());

  if (!map_pcd_path_.empty()) {
    loadMapPcd(map_pcd_path_);
  } else {
    // Default fallback to package share directory maps/robocon2026_field.pcd
    try {
      std::string pkg_share = ament_index_cpp::get_package_share_directory("lidar_perception_system");
      std::string default_pcd = (std::filesystem::path(pkg_share) / "maps" / "robocon2026_field.pcd").string();
      loadMapPcd(default_pcd);
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Failed to locate package share directory: %s", e.what());
    }
  }

  RCLCPP_INFO(this->get_logger(), "DynamicObjDetector initialized.");
}

void DynamicObjDetector::loadParameters()
{
  voxel_leaf_size_ = this->declare_parameter<double>("voxel_leaf_size", 0.05);
  bg_subtraction_radius_ = this->declare_parameter<double>("bg_subtraction_radius", 0.10);
  moving_bucket_z_min_ = this->declare_parameter<double>("moving_bucket_z_min", 1.2);
  moving_bucket_z_max_ = this->declare_parameter<double>("moving_bucket_z_max", 2.1);
  frame_accumulation_count_ = this->declare_parameter<int>("frame_accumulation_count", 3);
  cluster_tolerance_ = this->declare_parameter<double>("cluster_tolerance", 0.20);
  min_cluster_size_ = this->declare_parameter<int>("min_cluster_size", 20);
  max_cluster_size_ = this->declare_parameter<int>("max_cluster_size", 10000);
  input_topic_ = this->declare_parameter<std::string>("input_topic", "/livox/lidar");
  target_frame_ = this->declare_parameter<std::string>("target_frame", "map");
  map_pcd_path_ = this->declare_parameter<std::string>("map_pcd_path", "");

  enable_clear_bucket_fallback_ = this->declare_parameter<bool>("enable_clear_bucket_fallback", true);
  fallback_base_z_min_ = this->declare_parameter<double>("fallback_base_z_min", 0.1);
  fallback_base_z_max_ = this->declare_parameter<double>("fallback_base_z_max", 1.0);
  fallback_virtual_z_offset_ = this->declare_parameter<double>("fallback_virtual_z_offset", 0.80);
}

bool DynamicObjDetector::loadMapPcd(const std::string & pcd_path)
{
  std::string target_path = pcd_path;

  if (!std::filesystem::exists(target_path)) {
    // Try resolving relative to package share directory
    try {
      std::string pkg_share = ament_index_cpp::get_package_share_directory("lidar_perception_system");
      std::string share_pcd = (std::filesystem::path(pkg_share) / "maps" / "robocon2026_field.pcd").string();
      if (std::filesystem::exists(share_pcd)) {
        RCLCPP_INFO(this->get_logger(), "Resolved map PCD path to package share: %s", share_pcd.c_str());
        target_path = share_pcd;
      }
    } catch (const std::exception &) {}
  }

  if (!std::filesystem::exists(target_path)) {
    RCLCPP_WARN(this->get_logger(), "Static map PCD path does not exist: %s", pcd_path.c_str());
    has_map_ = false;
    return false;
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(target_path, *map_cloud_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load static map PCD file: %s", target_path.c_str());
    has_map_ = false;
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Loaded static map PCD with %zu points.", map_cloud_->size());
  map_kdtree_->setInputCloud(map_cloud_);
  has_map_ = true;

  // Cache and publish map cloud for RViz visualization
  pcl::toROSMsg(*map_cloud_, cached_map_msg_);
  cached_map_msg_.header.frame_id = target_frame_;
  cached_map_msg_.header.stamp = this->now();
  pub_map_cloud_->publish(cached_map_msg_);

  // Create timer to periodically publish map cloud (every 1s) for late-connecting RViz2
  map_pub_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    [this]() {
      if (has_map_) {
        cached_map_msg_.header.stamp = this->now();
        pub_map_cloud_->publish(cached_map_msg_);
      }
    });

  return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DynamicObjDetector::performBackgroundSubtraction(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud)
{
  if (!has_map_ || map_cloud_->empty()) {
    return input_cloud;  // Pass through if no map
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr fg_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> point_idx;
  std::vector<float> point_sqr_dist;

  for (const auto & pt : input_cloud->points) {
    if (map_kdtree_->radiusSearch(pt, bg_subtraction_radius_, point_idx, point_sqr_dist) == 0) {
      fg_cloud->push_back(pt);  // Point is far from static map -> Foreground
    }
  }

  return fg_cloud;
}

void DynamicObjDetector::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // 1. Transform current cloud to target_frame_ ("map")
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
      "DynamicObjDetector: Transform error to %s: %s", target_frame_.c_str(), ex.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_current(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud_transformed, *cloud_current);

  // 2. Buffer accumulation for non-repetitive scan (Mid-360)
  cloud_buffer_.push_back(cloud_current);
  if (static_cast<int>(cloud_buffer_.size()) > frame_accumulation_count_) {
    cloud_buffer_.pop_front();
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto & cloud_frame : cloud_buffer_) {
    *accumulated_cloud += *cloud_frame;
  }

  if (accumulated_cloud->empty()) {
    return;
  }

  // 3. VoxelGrid downsampling
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(accumulated_cloud);
  voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel_filter.filter(*voxelized_cloud);

  // 4. Background subtraction
  pcl::PointCloud<pcl::PointXYZ>::Ptr fg_cloud = performBackgroundSubtraction(voxelized_cloud);

  // Publish dynamic cloud for debugging
  sensor_msgs::msg::PointCloud2 dynamic_msg;
  pcl::toROSMsg(*fg_cloud, dynamic_msg);
  dynamic_msg.header.frame_id = target_frame_;
  dynamic_msg.header.stamp = msg->header.stamp;
  pub_dynamic_cloud_->publish(dynamic_msg);

  visualization_msgs::msg::MarkerArray marker_array;

  // 5. Detect Moving Bucket (Direct Height Filter Z=1.2m ~ 2.1m)
  pcl::PointCloud<pcl::PointXYZ>::Ptr bucket_candidates(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(fg_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(moving_bucket_z_min_, moving_bucket_z_max_);
  pass.filter(*bucket_candidates);

  bool bucket_detected = false;
  Eigen::Vector4f centroid;

  if (bucket_candidates->size() >= static_cast<size_t>(min_cluster_size_)) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(bucket_candidates);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(bucket_candidates);
    ec.extract(cluster_indices);

    if (!cluster_indices.empty()) {
      // Find largest cluster
      size_t max_size = 0;
      size_t largest_idx = 0;
      for (size_t i = 0; i < cluster_indices.size(); ++i) {
        if (cluster_indices[i].indices.size() > max_size) {
          max_size = cluster_indices[i].indices.size();
          largest_idx = i;
        }
      }

      pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (int idx : cluster_indices[largest_idx].indices) {
        largest_cluster->push_back((*bucket_candidates)[idx]);
      }

      pcl::compute3DCentroid(*largest_cluster, centroid);
      bucket_detected = true;

      // Marker: Red Sphere
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = msg->header.stamp;
      marker.header.frame_id = target_frame_;
      marker.ns = "dynamic_bucket";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = centroid[0];
      marker.pose.position.y = centroid[1];
      marker.pose.position.z = centroid[2];
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.35;
      marker.scale.y = 0.35;
      marker.scale.z = 0.35;
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.9f;
      marker.lifetime = rclcpp::Duration::from_seconds(0.5);
      marker_array.markers.push_back(marker);
    }
  }

  // 6. Fallback Logic for Clear Polycarbonate Bucket
  if (!bucket_detected && enable_clear_bucket_fallback_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr base_candidates(new pcl::PointCloud<pcl::PointXYZ>);
    pass.setInputCloud(fg_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(fallback_base_z_min_, fallback_base_z_max_);
    pass.filter(*base_candidates);

    if (base_candidates->size() >= static_cast<size_t>(min_cluster_size_)) {
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(base_candidates);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(cluster_tolerance_);
      ec.setMinClusterSize(min_cluster_size_);
      ec.setMaxClusterSize(max_cluster_size_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(base_candidates);
      ec.extract(cluster_indices);

      if (!cluster_indices.empty()) {
        size_t max_size = 0;
        size_t largest_idx = 0;
        for (size_t i = 0; i < cluster_indices.size(); ++i) {
          if (cluster_indices[i].indices.size() > max_size) {
            max_size = cluster_indices[i].indices.size();
            largest_idx = i;
          }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr base_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (int idx : cluster_indices[largest_idx].indices) {
          base_cluster->push_back((*base_candidates)[idx]);
        }

        pcl::compute3DCentroid(*base_cluster, centroid);
        // Apply virtual Z offset for transparent bucket estimation
        centroid[2] += static_cast<float>(fallback_virtual_z_offset_);
        bucket_detected = true;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Fallback triggered: Moving bucket estimated via base centroid + offset Z (+%.2fm)",
          fallback_virtual_z_offset_);

        // Marker: Orange Sphere (indicating fallback)
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = msg->header.stamp;
        marker.header.frame_id = target_frame_;
        marker.ns = "dynamic_bucket_fallback";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = centroid[0];
        marker.pose.position.y = centroid[1];
        marker.pose.position.z = centroid[2];
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.35;
        marker.scale.y = 0.35;
        marker.scale.z = 0.35;
        marker.color.r = 1.0f;
        marker.color.g = 0.5f;
        marker.color.b = 0.0f;
        marker.color.a = 0.9f;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker_array.markers.push_back(marker);
      }
    }
  }

  // 7. Broadcast TF: target_frame_ -> moving_bucket
  if (bucket_detected) {
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header.stamp = msg->header.stamp;
    tf_stamped.header.frame_id = target_frame_;
    tf_stamped.child_frame_id = "moving_bucket";
    tf_stamped.transform.translation.x = centroid[0];
    tf_stamped.transform.translation.y = centroid[1];
    tf_stamped.transform.translation.z = centroid[2];
    tf_stamped.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf_stamped);
  }

  pub_markers_->publish(marker_array);
}

}  // namespace lidar_perception_system

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar_perception_system::DynamicObjDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
