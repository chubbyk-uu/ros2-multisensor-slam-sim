#include "slam_robot_slam/laser_scan_preprocessor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace slam_robot_slam
{

bool hasValidLaserScanMetadata(
  const sensor_msgs::msg::LaserScan & scan)
{
  return std::isfinite(scan.angle_min) &&
         std::isfinite(scan.angle_max) &&
         std::isfinite(scan.angle_increment) &&
         scan.angle_increment != 0.0F &&
         std::isfinite(scan.range_min) &&
         std::isfinite(scan.range_max) &&
         scan.range_min >= 0.0F &&
         scan.range_max > scan.range_min;
}

std::vector<Point2D> projectLaserScan(
  const sensor_msgs::msg::LaserScan & scan,
  const double minimum_range,
  const double maximum_range,
  const std::size_t point_stride)
{
  if (!std::isfinite(minimum_range) ||
    !std::isfinite(maximum_range) ||
    minimum_range < 0.0 ||
    maximum_range <= minimum_range ||
    point_stride == 0U)
  {
    throw std::invalid_argument("Invalid laser projection parameters");
  }
  if (!hasValidLaserScanMetadata(scan)) {
    return {};
  }

  const double effective_minimum =
    std::max(minimum_range, static_cast<double>(scan.range_min));
  const double effective_maximum =
    std::min(maximum_range, static_cast<double>(scan.range_max));

  std::vector<Point2D> points;
  if (effective_minimum > effective_maximum || scan.ranges.empty()) {
    return points;
  }

  points.reserve((scan.ranges.size() + point_stride - 1U) / point_stride);
  for (std::size_t index = 0U; index < scan.ranges.size(); index += point_stride) {
    const double range = static_cast<double>(scan.ranges[index]);
    if (!std::isfinite(range) ||
      range < effective_minimum ||
      range > effective_maximum)
    {
      continue;
    }

    const double angle =
      static_cast<double>(scan.angle_min) +
      static_cast<double>(index) * static_cast<double>(scan.angle_increment);
    points.push_back(
      Point2D{
        static_cast<float>(range * std::cos(angle)),
        static_cast<float>(range * std::sin(angle))});
  }

  return points;
}

}  // namespace slam_robot_slam
