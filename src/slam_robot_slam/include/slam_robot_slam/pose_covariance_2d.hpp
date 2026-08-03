#ifndef SLAM_ROBOT_SLAM__POSE_COVARIANCE_2D_HPP_
#define SLAM_ROBOT_SLAM__POSE_COVARIANCE_2D_HPP_

#include <cstddef>

#include "slam_robot_slam/point2d.hpp"

namespace slam_robot_slam
{

struct PoseCovariance2DParameters
{
  double translation_stddev{0.05};
  double rotation_stddev{0.05};
  double degenerate_translation_stddev{0.30};
  double dead_reckoning_translation_stddev_per_sqrt_meter{0.10};
  double dead_reckoning_rotation_stddev_per_sqrt_radian{0.10};
};

struct PoseCovariance2D
{
  double xx{0.0};
  double xy{0.0};
  double yy{0.0};
  double yaw_yaw{0.0};
};

void validatePoseCovariance2DParameters(
  const PoseCovariance2DParameters & parameters);

PoseCovariance2D matchedPoseCovariance(
  std::size_t translation_observable_rank,
  const Point2D & weak_translation_direction,
  const PoseCovariance2DParameters & parameters);

PoseCovariance2D propagateDeadReckoningCovariance(
  const PoseCovariance2D & covariance,
  double translation_distance,
  double rotation_distance,
  const PoseCovariance2DParameters & parameters);

}  // namespace slam_robot_slam

#endif  // SLAM_ROBOT_SLAM__POSE_COVARIANCE_2D_HPP_
