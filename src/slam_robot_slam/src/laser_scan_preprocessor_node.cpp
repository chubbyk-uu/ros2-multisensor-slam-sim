#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "slam_robot_slam/laser_scan_preprocessor.hpp"

namespace slam_robot_slam
{

class LaserScanPreprocessorNode : public rclcpp::Node
{
public:
  LaserScanPreprocessorNode()
  : Node("laser_scan_preprocessor")
  {
    const auto input_topic =
      declare_parameter<std::string>("input_topic", "/scan");
    const auto output_topic =
      declare_parameter<std::string>(
      "output_topic", "/custom_slam/scan_points");
    minimum_range_ = declare_parameter<double>("minimum_range", 0.12);
    maximum_range_ = declare_parameter<double>("maximum_range", 12.0);
    const auto point_stride =
      declare_parameter<int64_t>("point_stride", 1);

    if (minimum_range_ < 0.0 || maximum_range_ <= minimum_range_) {
      throw std::invalid_argument(
              "minimum_range must be non-negative and less than maximum_range");
    }
    if (point_stride < 1) {
      throw std::invalid_argument("point_stride must be at least one");
    }
    point_stride_ = static_cast<std::size_t>(point_stride);

    point_cloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    laser_scan_subscription_ =
      create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic,
      rclcpp::SensorDataQoS(),
      std::bind(
        &LaserScanPreprocessorNode::laserScanCallback,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Laser preprocessing: %s -> %s, range [%.3f, %.3f] m, stride %zu",
      input_topic.c_str(),
      output_topic.c_str(),
      minimum_range_,
      maximum_range_,
      point_stride_);
  }

private:
  void laserScanCallback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
  {
    if (!hasValidLaserScanMetadata(*scan)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Ignoring LaserScan with invalid angular metadata");
      return;
    }

    const auto points = projectLaserScan(
      *scan, minimum_range_, maximum_range_, point_stride_);

    sensor_msgs::msg::PointCloud2 point_cloud;
    point_cloud.header = scan->header;
    point_cloud.height = 1U;
    point_cloud.width = static_cast<uint32_t>(points.size());
    point_cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(point_cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> output_x(point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> output_y(point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> output_z(point_cloud, "z");
    for (const auto & point : points) {
      *output_x = point.x;
      *output_y = point.y;
      *output_z = 0.0F;
      ++output_x;
      ++output_y;
      ++output_z;
    }

    point_cloud_publisher_->publish(point_cloud);
  }

  double minimum_range_;
  double maximum_range_;
  std::size_t point_stride_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    point_cloud_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
    laser_scan_subscription_;
};

}  // namespace slam_robot_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<slam_robot_slam::LaserScanPreprocessorNode>());
  rclcpp::shutdown();
  return 0;
}
