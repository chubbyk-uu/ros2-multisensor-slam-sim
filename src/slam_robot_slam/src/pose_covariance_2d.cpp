#include "slam_robot_slam/pose_covariance_2d.hpp"

#include <cmath>
#include <stdexcept>

namespace slam_robot_slam
{
namespace
{

bool isFinitePositive(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

void validatePoseCovariance2DParameters(
  const PoseCovariance2DParameters & parameters)
{
  if (!isFinitePositive(parameters.translation_stddev) ||
    !isFinitePositive(parameters.rotation_stddev) ||
    !isFinitePositive(parameters.degenerate_translation_stddev) ||
    parameters.degenerate_translation_stddev <
    parameters.translation_stddev ||
    !isFinitePositive(
      parameters.dead_reckoning_translation_stddev_per_sqrt_meter) ||
    !isFinitePositive(
      parameters.dead_reckoning_rotation_stddev_per_sqrt_radian))
  {
    throw std::invalid_argument("Invalid pose covariance parameters");
  }
}

PoseCovariance2D matchedPoseCovariance(
  const std::size_t translation_observable_rank,
  const Point2D & weak_translation_direction,
  const PoseCovariance2DParameters & parameters)
{
  validatePoseCovariance2DParameters(parameters);
  const double regular_variance =
    parameters.translation_stddev * parameters.translation_stddev;
  const double rotation_variance =
    parameters.rotation_stddev * parameters.rotation_stddev;
  PoseCovariance2D covariance{
    regular_variance, 0.0, regular_variance, rotation_variance};
  if (translation_observable_rank >= 2U) {
    return covariance;
  }

  const double degenerate_variance =
    parameters.degenerate_translation_stddev *
    parameters.degenerate_translation_stddev;
  if (translation_observable_rank == 0U) {
    covariance.xx = degenerate_variance;
    covariance.yy = degenerate_variance;
    return covariance;
  }
  if (!std::isfinite(weak_translation_direction.x) ||
    !std::isfinite(weak_translation_direction.y))
  {
    throw std::invalid_argument("Weak translation direction must be finite");
  }

  const double norm = std::hypot(
    weak_translation_direction.x, weak_translation_direction.y);
  if (norm <= 0.0) {
    throw std::invalid_argument("Weak translation direction must be nonzero");
  }
  const double weak_x = weak_translation_direction.x / norm;
  const double weak_y = weak_translation_direction.y / norm;
  const double additional_variance =
    degenerate_variance - regular_variance;
  covariance.xx += additional_variance * weak_x * weak_x;
  covariance.xy += additional_variance * weak_x * weak_y;
  covariance.yy += additional_variance * weak_y * weak_y;
  return covariance;
}

PoseCovariance2D propagateDeadReckoningCovariance(
  const PoseCovariance2D & covariance,
  const double translation_distance,
  const double rotation_distance,
  const PoseCovariance2DParameters & parameters)
{
  validatePoseCovariance2DParameters(parameters);
  if (!std::isfinite(covariance.xx) ||
    !std::isfinite(covariance.xy) ||
    !std::isfinite(covariance.yy) ||
    !std::isfinite(covariance.yaw_yaw) ||
    covariance.xx < 0.0 || covariance.yy < 0.0 ||
    covariance.yaw_yaw < 0.0 ||
    !std::isfinite(translation_distance) || translation_distance < 0.0 ||
    !std::isfinite(rotation_distance) || rotation_distance < 0.0)
  {
    throw std::invalid_argument("Invalid dead-reckoning covariance input");
  }

  const double translation_process_variance =
    parameters.dead_reckoning_translation_stddev_per_sqrt_meter *
    parameters.dead_reckoning_translation_stddev_per_sqrt_meter *
    translation_distance;
  const double rotation_process_variance =
    parameters.dead_reckoning_rotation_stddev_per_sqrt_radian *
    parameters.dead_reckoning_rotation_stddev_per_sqrt_radian *
    rotation_distance;
  return PoseCovariance2D{
    covariance.xx + translation_process_variance,
    covariance.xy,
    covariance.yy + translation_process_variance,
    covariance.yaw_yaw + rotation_process_variance};
}

}  // namespace slam_robot_slam
