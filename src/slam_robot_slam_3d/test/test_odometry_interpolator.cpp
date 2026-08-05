#include <cmath>
#include <cstdint>
#include <deque>

#include <gtest/gtest.h>

#include "slam_robot_slam_3d/odometry_interpolator.hpp"

namespace slam_robot_slam_3d
{
namespace
{

nav_msgs::msg::Odometry makeOdometry(
  double time, double x, double yaw, double linear_velocity)
{
  nav_msgs::msg::Odometry message;
  message.header.stamp = rclcpp::Time(
    static_cast<std::int64_t>(time * 1.0e9), RCL_ROS_TIME);
  message.header.frame_id = "odom";
  message.child_frame_id = "base_footprint";
  message.pose.pose.position.x = x;
  message.pose.pose.orientation.z = std::sin(0.5 * yaw);
  message.pose.pose.orientation.w = std::cos(0.5 * yaw);
  message.twist.twist.linear.x = linear_velocity;
  message.pose.covariance[0] = x;
  return message;
}

TEST(OdometryInterpolator, InterpolatesPoseTwistAndCovarianceAtCloudTime)
{
  std::deque<nav_msgs::msg::Odometry> buffer{
    makeOdometry(1.00, 0.0, 0.0, 1.0),
    makeOdometry(1.02, 0.2, 0.2, 3.0)};

  const auto result = interpolateOdometry(
    buffer, rclcpp::Time(1010000000LL, RCL_ROS_TIME), 0.02);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(rclcpp::Time(result->header.stamp).nanoseconds(), 1010000000LL);
  EXPECT_NEAR(result->pose.pose.position.x, 0.1, 1.0e-9);
  const double yaw = 2.0 * std::atan2(
    result->pose.pose.orientation.z, result->pose.pose.orientation.w);
  EXPECT_NEAR(yaw, 0.1, 1.0e-9);
  EXPECT_NEAR(result->twist.twist.linear.x, 2.0, 1.0e-9);
  EXPECT_NEAR(result->pose.covariance[0], 0.1, 1.0e-9);
}

TEST(OdometryInterpolator, WaitsForFutureBracketAndRejectsStaleSamples)
{
  std::deque<nav_msgs::msg::Odometry> buffer{
    makeOdometry(1.00, 0.0, 0.0, 1.0)};
  const rclcpp::Time cloud_time(1010000000LL, RCL_ROS_TIME);
  EXPECT_FALSE(interpolateOdometry(buffer, cloud_time, 0.05).has_value());

  buffer.push_back(makeOdometry(1.20, 0.2, 0.0, 1.0));
  EXPECT_FALSE(interpolateOdometry(buffer, cloud_time, 0.05).has_value());
}

TEST(OdometryInterpolator, AcceptsAnExactSampleWithoutFutureData)
{
  std::deque<nav_msgs::msg::Odometry> buffer{
    makeOdometry(1.00, 0.3, 0.1, 1.0)};

  const auto result = interpolateOdometry(
    buffer, rclcpp::Time(1000000000LL, RCL_ROS_TIME), 0.01);

  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->pose.pose.position.x, 0.3);
}

}  // namespace
}  // namespace slam_robot_slam_3d
