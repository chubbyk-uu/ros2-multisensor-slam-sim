#include "slam_robot_slam_3d/odometry_interpolator.hpp"

#include <cmath>
#include <cstddef>

#include <Eigen/Geometry>

namespace slam_robot_slam_3d
{
namespace
{

double interpolate(double first, double second, double ratio)
{
  return first + ratio * (second - first);
}

}  // namespace

std::optional<nav_msgs::msg::Odometry> interpolateOdometry(
  const std::deque<nav_msgs::msg::Odometry> & buffer,
  const rclcpp::Time & target_time,
  double maximum_sample_age)
{
  if (buffer.empty() || !std::isfinite(maximum_sample_age) ||
    maximum_sample_age <= 0.0)
  {
    return std::nullopt;
  }

  const nav_msgs::msg::Odometry * before = nullptr;
  const nav_msgs::msg::Odometry * after = nullptr;
  for (const auto & sample : buffer) {
    const rclcpp::Time sample_time(sample.header.stamp);
    if (sample_time <= target_time &&
      (before == nullptr ||
      rclcpp::Time(before->header.stamp) < sample_time))
    {
      before = &sample;
    }
    if (sample_time >= target_time &&
      (after == nullptr || sample_time < rclcpp::Time(after->header.stamp)))
    {
      after = &sample;
    }
  }
  if (before == nullptr || after == nullptr) {
    return std::nullopt;
  }

  const rclcpp::Time before_time(before->header.stamp);
  const rclcpp::Time after_time(after->header.stamp);
  const double before_age = (target_time - before_time).seconds();
  const double after_age = (after_time - target_time).seconds();
  if (before_age > maximum_sample_age || after_age > maximum_sample_age) {
    return std::nullopt;
  }
  if (before_time == after_time) {
    auto result = *before;
    result.header.stamp = target_time;
    return result;
  }
  if (before->header.frame_id != after->header.frame_id ||
    before->child_frame_id != after->child_frame_id)
  {
    return std::nullopt;
  }

  const double ratio = before_age / (after_time - before_time).seconds();
  const auto & first_pose = before->pose.pose;
  const auto & second_pose = after->pose.pose;
  Eigen::Quaterniond first_rotation(
    first_pose.orientation.w, first_pose.orientation.x,
    first_pose.orientation.y, first_pose.orientation.z);
  Eigen::Quaterniond second_rotation(
    second_pose.orientation.w, second_pose.orientation.x,
    second_pose.orientation.y, second_pose.orientation.z);
  if (!first_rotation.coeffs().allFinite() ||
    !second_rotation.coeffs().allFinite() ||
    first_rotation.norm() < 1.0e-6 || second_rotation.norm() < 1.0e-6)
  {
    return std::nullopt;
  }
  first_rotation.normalize();
  second_rotation.normalize();
  const Eigen::Quaterniond rotation = first_rotation.slerp(ratio, second_rotation);

  auto result = *before;
  result.header.stamp = target_time;
  result.pose.pose.position.x = interpolate(
    first_pose.position.x, second_pose.position.x, ratio);
  result.pose.pose.position.y = interpolate(
    first_pose.position.y, second_pose.position.y, ratio);
  result.pose.pose.position.z = interpolate(
    first_pose.position.z, second_pose.position.z, ratio);
  result.pose.pose.orientation.x = rotation.x();
  result.pose.pose.orientation.y = rotation.y();
  result.pose.pose.orientation.z = rotation.z();
  result.pose.pose.orientation.w = rotation.w();

  const auto & first_twist = before->twist.twist;
  const auto & second_twist = after->twist.twist;
  result.twist.twist.linear.x = interpolate(
    first_twist.linear.x, second_twist.linear.x, ratio);
  result.twist.twist.linear.y = interpolate(
    first_twist.linear.y, second_twist.linear.y, ratio);
  result.twist.twist.linear.z = interpolate(
    first_twist.linear.z, second_twist.linear.z, ratio);
  result.twist.twist.angular.x = interpolate(
    first_twist.angular.x, second_twist.angular.x, ratio);
  result.twist.twist.angular.y = interpolate(
    first_twist.angular.y, second_twist.angular.y, ratio);
  result.twist.twist.angular.z = interpolate(
    first_twist.angular.z, second_twist.angular.z, ratio);
  for (std::size_t index = 0U; index < result.pose.covariance.size(); ++index) {
    result.pose.covariance[index] = interpolate(
      before->pose.covariance[index], after->pose.covariance[index], ratio);
    result.twist.covariance[index] = interpolate(
      before->twist.covariance[index], after->twist.covariance[index], ratio);
  }
  return result;
}

}  // namespace slam_robot_slam_3d
