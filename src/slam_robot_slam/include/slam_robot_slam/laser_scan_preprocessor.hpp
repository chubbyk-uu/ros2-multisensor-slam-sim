#ifndef SLAM_ROBOT_SLAM__LASER_SCAN_PREPROCESSOR_HPP_
#define SLAM_ROBOT_SLAM__LASER_SCAN_PREPROCESSOR_HPP_

#include <cstddef>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace slam_robot_slam
{

struct Point2D
{
  float x;
  float y;
};

std::vector<Point2D> projectLaserScan(
  const sensor_msgs::msg::LaserScan & scan,
  double minimum_range,
  double maximum_range,
  std::size_t point_stride);

}  // namespace slam_robot_slam

#endif  // SLAM_ROBOT_SLAM__LASER_SCAN_PREPROCESSOR_HPP_
