#ifndef SLAM_ROBOT_SLAM__SCAN_POINT2D_HPP_
#define SLAM_ROBOT_SLAM__SCAN_POINT2D_HPP_

#include <cstddef>

#include "slam_robot_slam/point2d.hpp"

namespace slam_robot_slam
{

struct ScanPoint2D
{
  Point2D point;
  std::size_t beam_index{0U};
  float range{0.0F};
};

}  // namespace slam_robot_slam

#endif  // SLAM_ROBOT_SLAM__SCAN_POINT2D_HPP_
