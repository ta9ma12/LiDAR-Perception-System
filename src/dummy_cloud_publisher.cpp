#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/static_transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std::chrono_literals;

class DummyCloudPublisher : public rclcpp::Node
{
public:
  DummyCloudPublisher()
  : Node("dummy_cloud_publisher")
  {
    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10);

    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publishStaticTFs();

    timer_ = this->create_wall_timer(
      100ms, std::bind(&DummyCloudPublisher::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Dummy Cloud Publisher started (10 Hz). Publishing to /livox/lidar");
  }

private:
  void publishStaticTFs()
  {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    // map -> base_link
    geometry_msgs::msg::TransformStamped tf_map_base;
    tf_map_base.header.stamp = this->now();
    tf_map_base.header.frame_id = "map";
    tf_map_base.child_frame_id = "base_link";
    tf_map_base.transform.translation.x = 0.0;
    tf_map_base.transform.translation.y = 0.0;
    tf_map_base.transform.translation.z = 0.2;
    tf_map_base.transform.rotation.w = 1.0;
    transforms.push_back(tf_map_base);

    // base_link -> livox_frame
    geometry_msgs::msg::TransformStamped tf_base_livox;
    tf_base_livox.header.stamp = this->now();
    tf_base_livox.header.frame_id = "base_link";
    tf_base_livox.child_frame_id = "livox_frame";
    tf_base_livox.transform.translation.x = 0.1;
    tf_base_livox.transform.translation.y = 0.0;
    tf_base_livox.transform.translation.z = 0.5;
    tf_base_livox.transform.rotation.w = 1.0;
    transforms.push_back(tf_base_livox);

    static_tf_broadcaster_->sendTransform(transforms);
  }

  void addCluster(
    pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    double cx, double cy, double cz,
    double size_x, double size_y, double size_z,
    int num_points)
  {
    for (int i = 0; i < num_points; ++i) {
      double rx = ((double)rand() / RAND_MAX - 0.5) * size_x;
      double ry = ((double)rand() / RAND_MAX - 0.5) * size_y;
      double rz = ((double)rand() / RAND_MAX - 0.5) * size_z;
      cloud->push_back(pcl::PointXYZ(cx + rx, cy + ry, cz + rz));
    }
  }

  void timerCallback()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Ground plane
    for (double x = -3.0; x <= 3.0; x += 0.2) {
      for (double y = -3.0; y <= 3.0; y += 0.2) {
        cloud->push_back(pcl::PointXYZ(x, y, 0.0));
      }
    }

    // Fixed Bucket 1 (x:1.0, y:1.0, z:0.1)
    addCluster(cloud, 1.0, 1.0, 0.1, 0.3, 0.3, 0.2, 80);

    // Fixed Bucket 2 (x:2.5, y:-1.5, z:0.75)
    addCluster(cloud, 2.5, -1.5, 0.75, 0.3, 0.3, 0.2, 80);

    // Fixed Bucket 3 (x:0.0, y:2.0, z:0.4)
    addCluster(cloud, 0.0, 2.0, 0.4, 0.3, 0.3, 0.2, 80);

    // Flag (x:0.0, y:0.0, z:2.9)
    addCluster(cloud, 0.0, 0.0, 2.9, 0.2, 0.2, 0.1, 50);

    // Dynamic Moving Bucket / Opponent Robot Base (Moving in circle)
    static double angle = 0.0;
    angle += 0.05;
    double robot_x = 2.0 * std::cos(angle);
    double robot_y = 2.0 * std::sin(angle);

    // Opponent robot base (z:0.5) - Triggers Fallback if bucket is clear!
    addCluster(cloud, robot_x, robot_y, 0.5, 0.6, 0.6, 0.4, 150);

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "map";

    pub_cloud_->publish(cloud_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}
