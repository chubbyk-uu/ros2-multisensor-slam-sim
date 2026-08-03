#ifndef SLAM_ROBOT_SLAM__LASER_SCAN_PREPROCESSOR_HPP_
#define SLAM_ROBOT_SLAM__LASER_SCAN_PREPROCESSOR_HPP_

#include <cstddef>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "slam_robot_slam/point2d.hpp"
#include "slam_robot_slam/scan_point2d.hpp"

namespace slam_robot_slam
{

bool hasValidLaserScanMetadata(
  const sensor_msgs::msg::LaserScan & scan);

std::vector<ScanPoint2D> projectOrderedLaserScan(
  const sensor_msgs::msg::LaserScan & scan,
  double minimum_range,
  double maximum_range,
  std::size_t point_stride);

std::vector<Point2D> projectLaserScan(
  const sensor_msgs::msg::LaserScan & scan,
  double minimum_range,
  double maximum_range,
  std::size_t point_stride);

}  // namespace slam_robot_slam

#endif  // SLAM_ROBOT_SLAM__LASER_SCAN_PREPROCESSOR_HPP_
