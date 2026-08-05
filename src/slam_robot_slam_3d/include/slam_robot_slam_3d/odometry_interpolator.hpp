#ifndef SLAM_ROBOT_SLAM_3D__ODOMETRY_INTERPOLATOR_HPP_
#define SLAM_ROBOT_SLAM_3D__ODOMETRY_INTERPOLATOR_HPP_

#include <deque>
#include <optional>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>

namespace slam_robot_slam_3d
{

std::optional<nav_msgs::msg::Odometry> interpolateOdometry(
  const std::deque<nav_msgs::msg::Odometry> & buffer,
  const rclcpp::Time & target_time,
  double maximum_sample_age);

}  // namespace slam_robot_slam_3d

#endif  // SLAM_ROBOT_SLAM_3D__ODOMETRY_INTERPOLATOR_HPP_
