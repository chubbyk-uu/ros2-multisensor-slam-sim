// Copyright 2026 Jerry
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace slam_robot_gazebo
{

class SensorCovarianceAdapterNode : public rclcpp::Node
{
public:
  SensorCovarianceAdapterNode()
  : Node("sensor_covariance_adapter")
  {
    const auto wheel_input_topic = declare_parameter<std::string>(
      "wheel_input_topic", "/wheel/odom_raw");
    const auto wheel_output_topic = declare_parameter<std::string>(
      "wheel_output_topic", "/wheel/odom");
    const auto imu_input_topic = declare_parameter<std::string>(
      "imu_input_topic", "/imu/data_raw");
    const auto imu_output_topic = declare_parameter<std::string>(
      "imu_output_topic", "/imu/data");
    wheel_pose_translation_stddev_ = declarePositiveParameter(
      "wheel.pose_translation_stddev", 0.05);
    wheel_pose_yaw_stddev_ = declarePositiveParameter(
      "wheel.pose_yaw_stddev", 0.05);
    wheel_linear_x_stddev_ = declarePositiveParameter(
      "wheel.linear_x_stddev", 0.02);
    wheel_linear_y_stddev_ = declarePositiveParameter(
      "wheel.linear_y_stddev", 0.005);
    wheel_angular_z_stddev_ = declarePositiveParameter(
      "wheel.angular_z_stddev", 0.02);
    imu_angular_velocity_stddev_ = declarePositiveParameter(
      "imu.angular_velocity_stddev", 0.0021);
    imu_linear_acceleration_stddev_ = declarePositiveParameter(
      "imu.linear_acceleration_stddev", 0.0207);

    wheel_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      wheel_output_topic, rclcpp::QoS(20));
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      imu_output_topic, rclcpp::SensorDataQoS());
    wheel_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_input_topic,
      rclcpp::QoS(20),
      std::bind(
        &SensorCovarianceAdapterNode::wheelCallback,
        this,
        std::placeholders::_1));
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_input_topic,
      rclcpp::SensorDataQoS(),
      std::bind(
        &SensorCovarianceAdapterNode::imuCallback,
        this,
        std::placeholders::_1));
  }

private:
  double declarePositiveParameter(const std::string & name, const double value)
  {
    const double parameter = declare_parameter<double>(name, value);
    if (!std::isfinite(parameter) || parameter <= 0.0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return parameter;
  }

  static double variance(const double standard_deviation)
  {
    return standard_deviation * standard_deviation;
  }

  void wheelCallback(const nav_msgs::msg::Odometry::SharedPtr input)
  {
    nav_msgs::msg::Odometry output = *input;
    output.pose.covariance.fill(0.0);
    output.pose.covariance[0] = variance(wheel_pose_translation_stddev_);
    output.pose.covariance[7] = variance(wheel_pose_translation_stddev_);
    output.pose.covariance[14] = 1.0e6;
    output.pose.covariance[21] = 1.0e6;
    output.pose.covariance[28] = 1.0e6;
    output.pose.covariance[35] = variance(wheel_pose_yaw_stddev_);

    output.twist.covariance.fill(0.0);
    output.twist.covariance[0] = variance(wheel_linear_x_stddev_);
    output.twist.covariance[7] = variance(wheel_linear_y_stddev_);
    output.twist.covariance[14] = 1.0e6;
    output.twist.covariance[21] = 1.0e6;
    output.twist.covariance[28] = 1.0e6;
    output.twist.covariance[35] = variance(wheel_angular_z_stddev_);
    wheel_publisher_->publish(output);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr input)
  {
    sensor_msgs::msg::Imu output = *input;
    output.orientation_covariance.fill(0.0);
    output.orientation_covariance[0] = -1.0;
    output.angular_velocity_covariance.fill(0.0);
    output.linear_acceleration_covariance.fill(0.0);
    for (std::size_t index = 0U; index < 3U; ++index) {
      output.angular_velocity_covariance[index * 3U + index] =
        variance(imu_angular_velocity_stddev_);
      output.linear_acceleration_covariance[index * 3U + index] =
        variance(imu_linear_acceleration_stddev_);
    }
    imu_publisher_->publish(output);
  }

  double wheel_pose_translation_stddev_{0.0};
  double wheel_pose_yaw_stddev_{0.0};
  double wheel_linear_x_stddev_{0.0};
  double wheel_linear_y_stddev_{0.0};
  double wheel_angular_z_stddev_{0.0};
  double imu_angular_velocity_stddev_{0.0};
  double imu_linear_acceleration_stddev_{0.0};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

}  // namespace slam_robot_gazebo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<slam_robot_gazebo::SensorCovarianceAdapterNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("sensor_covariance_adapter"),
      "Fatal covariance adapter error: %s",
      error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
