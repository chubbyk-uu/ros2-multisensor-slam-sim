#include "slam_robot_slam_3d/scan_to_map_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>
#include <pcl/search/kdtree.h>

namespace slam_robot_slam_3d
{
namespace
{

bool cloudIsFinite(const pcl::PointCloud<pcl::PointXYZI> & cloud)
{
  return std::all_of(
    cloud.begin(), cloud.end(),
    [](const auto & point) {return pcl::isFinite(point);});
}

double rotationAngle(const Eigen::Matrix3d & rotation)
{
  return Eigen::AngleAxisd(rotation).angle();
}

}  // namespace

bool ScanToMapResult::success() const
{
  return status == ScanToMapStatus::kSuccess;
}

ScanToMapMatcher::ScanToMapMatcher(ScanToMapMatcherParameters parameters)
: parameters_(std::move(parameters))
{
  validateParameters();
}

ScanToMapResult ScanToMapMatcher::match(
  const pcl::PointCloud<pcl::PointXYZI> & scan,
  const pcl::PointCloud<pcl::PointXYZI> & local_map,
  const Eigen::Isometry3d & initial_pose) const
{
  ScanToMapResult result;
  result.pose = initial_pose;
  if (!initial_pose.matrix().allFinite() || !cloudIsFinite(scan) ||
    !cloudIsFinite(local_map))
  {
    result.status = ScanToMapStatus::kInvalidInput;
    return result;
  }
  if (scan.size() < parameters_.minimum_points ||
    local_map.size() < parameters_.minimum_points)
  {
    result.status = ScanToMapStatus::kTooFewPoints;
    return result;
  }

  const auto scan_pointer = scan.makeShared();
  const auto map_pointer = local_map.makeShared();
  pcl::GeneralizedIterativeClosestPoint<
    pcl::PointXYZI, pcl::PointXYZI> matcher;
  matcher.setInputSource(scan_pointer);
  matcher.setInputTarget(map_pointer);
  matcher.setMaximumIterations(parameters_.maximum_iterations);
  matcher.setMaximumOptimizerIterations(
    parameters_.maximum_optimizer_iterations);
  matcher.setCorrespondenceRandomness(parameters_.correspondence_randomness);
  matcher.setMaxCorrespondenceDistance(
    parameters_.maximum_correspondence_distance);
  matcher.setTransformationEpsilon(parameters_.transformation_epsilon);
  matcher.setRotationEpsilon(parameters_.rotation_epsilon);
  matcher.setEuclideanFitnessEpsilon(
    parameters_.euclidean_fitness_epsilon);

  pcl::PointCloud<pcl::PointXYZI> aligned_scan;
  matcher.align(aligned_scan, initial_pose.matrix().cast<float>());
  if (!matcher.hasConverged()) {
    result.status = ScanToMapStatus::kNotConverged;
    return result;
  }

  const Eigen::Matrix4d final_matrix =
    matcher.getFinalTransformation().cast<double>();
  if (!final_matrix.allFinite()) {
    result.status = ScanToMapStatus::kInvalidInput;
    return result;
  }
  result.pose.matrix() = final_matrix;

  const Eigen::Isometry3d correction = initial_pose.inverse() * result.pose;
  result.correction_translation = correction.translation().norm();
  result.correction_rotation = rotationAngle(correction.rotation());
  if (result.correction_translation >
    parameters_.maximum_correction_translation ||
    result.correction_rotation > parameters_.maximum_correction_rotation)
  {
    result.status = ScanToMapStatus::kCorrectionTooLarge;
    return result;
  }

  pcl::search::KdTree<pcl::PointXYZI> search;
  search.setInputCloud(map_pointer);
  const double maximum_distance_squared =
    parameters_.maximum_correspondence_distance *
    parameters_.maximum_correspondence_distance;
  double squared_error_sum = 0.0;
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  for (const auto & point : aligned_scan) {
    if (search.nearestKSearch(point, 1, indices, squared_distances) == 1 &&
      squared_distances.front() <= maximum_distance_squared)
    {
      ++result.correspondence_count;
      squared_error_sum += squared_distances.front();
    }
  }
  if (result.correspondence_count < parameters_.minimum_correspondences) {
    result.status = ScanToMapStatus::kInsufficientCorrespondences;
    return result;
  }
  result.rmse = std::sqrt(
    squared_error_sum / static_cast<double>(result.correspondence_count));
  if (!std::isfinite(result.rmse) || result.rmse > parameters_.maximum_rmse) {
    result.status = ScanToMapStatus::kFitnessTooHigh;
    return result;
  }

  result.status = ScanToMapStatus::kSuccess;
  return result;
}

const ScanToMapMatcherParameters & ScanToMapMatcher::parameters() const
{
  return parameters_;
}

void ScanToMapMatcher::validateParameters() const
{
  if (parameters_.maximum_iterations <= 0 ||
    parameters_.maximum_optimizer_iterations <= 0)
  {
    throw std::invalid_argument("GICP iteration limits must be positive");
  }
  if (parameters_.correspondence_randomness < 5) {
    throw std::invalid_argument("correspondence_randomness must be at least 5");
  }
  if (!std::isfinite(parameters_.maximum_correspondence_distance) ||
    parameters_.maximum_correspondence_distance <= 0.0)
  {
    throw std::invalid_argument(
            "maximum_correspondence_distance must be finite and positive");
  }
  if (!std::isfinite(parameters_.transformation_epsilon) ||
    parameters_.transformation_epsilon <= 0.0 ||
    !std::isfinite(parameters_.rotation_epsilon) ||
    parameters_.rotation_epsilon <= 0.0 ||
    !std::isfinite(parameters_.euclidean_fitness_epsilon) ||
    parameters_.euclidean_fitness_epsilon <= 0.0)
  {
    throw std::invalid_argument("GICP convergence tolerances must be positive");
  }
  if (parameters_.minimum_points <
    static_cast<std::size_t>(parameters_.correspondence_randomness) ||
    parameters_.minimum_correspondences == 0U)
  {
    throw std::invalid_argument("point and correspondence limits are invalid");
  }
  if (!std::isfinite(parameters_.maximum_rmse) ||
    parameters_.maximum_rmse <= 0.0 ||
    !std::isfinite(parameters_.maximum_correction_translation) ||
    parameters_.maximum_correction_translation <= 0.0 ||
    !std::isfinite(parameters_.maximum_correction_rotation) ||
    parameters_.maximum_correction_rotation <= 0.0)
  {
    throw std::invalid_argument("registration acceptance limits must be positive");
  }
}

const char * toString(ScanToMapStatus status)
{
  switch (status) {
    case ScanToMapStatus::kSuccess:
      return "success";
    case ScanToMapStatus::kInvalidInput:
      return "invalid_input";
    case ScanToMapStatus::kTooFewPoints:
      return "too_few_points";
    case ScanToMapStatus::kNotConverged:
      return "not_converged";
    case ScanToMapStatus::kInsufficientCorrespondences:
      return "insufficient_correspondences";
    case ScanToMapStatus::kFitnessTooHigh:
      return "fitness_too_high";
    case ScanToMapStatus::kCorrectionTooLarge:
      return "correction_too_large";
  }
  return "unknown";
}

}  // namespace slam_robot_slam_3d
