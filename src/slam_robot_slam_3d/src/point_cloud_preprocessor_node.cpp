#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include "slam_robot_slam_3d/point_cloud_preprocessor.hpp"

namespace slam_robot_slam_3d
{
namespace
{

diagnostic_msgs::msg::KeyValue makeValue(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

}  // namespace

class PointCloudPreprocessorNode : public rclcpp::Node
{
public:
  PointCloudPreprocessorNode()
  : Node("point_cloud_preprocessor_3d"),
    preprocessor_(declareParameters())
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/lidar_3d/points");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/custom_slam_3d/points_filtered");
    if (input_topic_.empty() || output_topic_.empty()) {
      throw std::invalid_argument("input_topic and output_topic must not be empty");
    }

    output_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());
    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/custom_slam_3d/preprocessing_diagnostics", 10);
    input_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(
        &PointCloudPreprocessorNode::pointCloudCallback, this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "3D preprocessing %s -> %s, range=[%.2f, %.2f] m, voxel=%.3f m, "
      "low-return=%s, statistical-outlier=%s",
      input_topic_.c_str(), output_topic_.c_str(),
      preprocessor_.parameters().minimum_range,
      preprocessor_.parameters().maximum_range,
      preprocessor_.parameters().voxel_leaf_size,
      preprocessor_.parameters().ground_filter_enabled ? "on" : "off",
      preprocessor_.parameters().outlier_filter_enabled ? "on" : "off");
  }

private:
  PointCloudPreprocessorParameters declareParameters()
  {
    PointCloudPreprocessorParameters parameters;
    parameters.minimum_range = declare_parameter<double>(
      "minimum_range", parameters.minimum_range);
    parameters.maximum_range = declare_parameter<double>(
      "maximum_range", parameters.maximum_range);
    parameters.voxel_leaf_size = declare_parameter<double>(
      "voxel_leaf_size", parameters.voxel_leaf_size);
    parameters.self_filter_enabled = declare_parameter<bool>(
      "self_filter.enabled", parameters.self_filter_enabled);
    parameters.self_min_x = declare_parameter<double>(
      "self_filter.min_x", parameters.self_min_x);
    parameters.self_max_x = declare_parameter<double>(
      "self_filter.max_x", parameters.self_max_x);
    parameters.self_min_y = declare_parameter<double>(
      "self_filter.min_y", parameters.self_min_y);
    parameters.self_max_y = declare_parameter<double>(
      "self_filter.max_y", parameters.self_max_y);
    parameters.self_min_z = declare_parameter<double>(
      "self_filter.min_z", parameters.self_min_z);
    parameters.self_max_z = declare_parameter<double>(
      "self_filter.max_z", parameters.self_max_z);
    parameters.ground_filter_enabled = declare_parameter<bool>(
      "ground_filter.enabled", parameters.ground_filter_enabled);
    parameters.ground_filter_maximum_z = declare_parameter<double>(
      "ground_filter.maximum_z", parameters.ground_filter_maximum_z);
    parameters.outlier_filter_enabled = declare_parameter<bool>(
      "outlier_filter.enabled", parameters.outlier_filter_enabled);
    parameters.outlier_filter_mean_k = declare_parameter<int>(
      "outlier_filter.mean_k", parameters.outlier_filter_mean_k);
    parameters.outlier_filter_standard_deviation_multiplier =
      declare_parameter<double>(
      "outlier_filter.standard_deviation_multiplier",
      parameters.outlier_filter_standard_deviation_multiplier);
    return parameters;
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    const auto started = std::chrono::steady_clock::now();
    try {
      pcl::PointCloud<pcl::PointXYZI> input;
      pcl::fromROSMsg(*message, input);
      PointCloudPreprocessingStatistics statistics;
      auto output = preprocessor_.process(input, &statistics);

      sensor_msgs::msg::PointCloud2 output_message;
      pcl::toROSMsg(output, output_message);
      output_message.header = message->header;
      output_publisher_->publish(output_message);

      const auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
      publishDiagnostics(message->header, statistics, elapsed);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Rejected PointCloud2 input: %s", error.what());
    }
  }

  void publishDiagnostics(
    const std_msgs::msg::Header & header,
    const PointCloudPreprocessingStatistics & statistics,
    double elapsed_milliseconds)
  {
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header = header;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name = "custom_slam_3d/point_cloud_preprocessor";
    status.hardware_id = header.frame_id;
    status.message = "OK";
    status.values.push_back(makeValue(
      "input_points", std::to_string(statistics.input_points)));
    status.values.push_back(makeValue(
      "finite_points", std::to_string(statistics.finite_points)));
    status.values.push_back(makeValue(
      "range_filtered_points",
      std::to_string(statistics.range_filtered_points)));
    status.values.push_back(makeValue(
      "self_filter_passed_points",
      std::to_string(statistics.self_filter_passed_points)));
    status.values.push_back(makeValue(
      "ground_filter_passed_points",
      std::to_string(statistics.ground_filter_passed_points)));
    status.values.push_back(makeValue(
      "outlier_filter_passed_points",
      std::to_string(statistics.outlier_filter_passed_points)));
    status.values.push_back(makeValue(
      "output_points", std::to_string(statistics.output_points)));
    status.values.push_back(makeValue(
      "processing_ms", std::to_string(elapsed_milliseconds)));
    message.status.push_back(std::move(status));
    diagnostics_publisher_->publish(message);
  }

  PointCloudPreprocessor preprocessor_;
  std::string input_topic_;
  std::string output_topic_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;
};

}  // namespace slam_robot_slam_3d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<slam_robot_slam_3d::PointCloudPreprocessorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("point_cloud_preprocessor_3d"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
