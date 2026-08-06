#include "slam_robot_slam_3d/scan_to_map_matcher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include <Eigen/Eigenvalues>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>
#include <pcl/search/kdtree.h>

namespace slam_robot_slam_3d
{
namespace
{

constexpr double kMinimumHorizontalNormalSquared = 0.01;

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

class ObservableGicp
  : public pcl::GeneralizedIterativeClosestPoint<
    pcl::PointXYZI, pcl::PointXYZI>
{
public:
  const MatricesVector & targetCovariances() const
  {
    return *target_covariances_;
  }
};

}  // namespace

class ScanToMapMatcher::Impl
{
public:
  explicit Impl(const ScanToMapMatcherParameters & parameters)
  {
    matcher.setMaximumIterations(parameters.maximum_iterations);
    matcher.setMaximumOptimizerIterations(parameters.maximum_optimizer_iterations);
    matcher.setCorrespondenceRandomness(parameters.correspondence_randomness);
    matcher.setMaxCorrespondenceDistance(parameters.maximum_correspondence_distance);
    matcher.setTransformationEpsilon(parameters.transformation_epsilon);
    matcher.setRotationEpsilon(parameters.rotation_epsilon);
    matcher.setEuclideanFitnessEpsilon(parameters.euclidean_fitness_epsilon);
  }

  ObservableGicp matcher;
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr target_cloud;
  std::uint64_t target_version{0U};
  std::vector<Eigen::Vector3d> target_normals;
  std::uint64_t target_normals_version{0U};
};

bool ScanToMapResult::success() const
{
  return status == ScanToMapStatus::kSuccess;
}

Eigen::Matrix2d ScanToMapResult::translationCovariance(
  double nominal_variance, double unobservable_variance) const
{
  if (!std::isfinite(nominal_variance) || nominal_variance <= 0.0 ||
    !std::isfinite(unobservable_variance) ||
    unobservable_variance < nominal_variance)
  {
    throw std::invalid_argument("translation variances are invalid");
  }
  if (translation_observable_rank <= 0) {
    return unobservable_variance * Eigen::Matrix2d::Identity();
  }
  const double weak_variance = nominal_variance +
    (1.0 - std::clamp(weak_translation_correction_scale, 0.0, 1.0)) *
    (unobservable_variance - nominal_variance);
  return nominal_variance * Eigen::Matrix2d::Identity() +
         (weak_variance - nominal_variance) *
         weak_translation_direction * weak_translation_direction.transpose();
}

ScanToMapMatcher::ScanToMapMatcher(ScanToMapMatcherParameters parameters)
: parameters_(std::move(parameters))
{
  validateParameters();
  implementation_ = std::make_unique<Impl>(parameters_);
}

ScanToMapMatcher::~ScanToMapMatcher() = default;

ScanToMapResult ScanToMapMatcher::match(
  const pcl::PointCloud<pcl::PointXYZI> & scan,
  const pcl::PointCloud<pcl::PointXYZI> & local_map,
  std::uint64_t local_map_version,
  const Eigen::Isometry3d & initial_pose)
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
  auto & matcher = implementation_->matcher;
  matcher.setInputSource(scan_pointer);
  if (!implementation_->target_cloud ||
    implementation_->target_version != local_map_version)
  {
    implementation_->target_cloud = local_map.makeShared();
    implementation_->target_version = local_map_version;
    matcher.setInputTarget(implementation_->target_cloud);
  } else {
    result.target_cache_reused = true;
  }

  pcl::PointCloud<pcl::PointXYZI> aligned_scan;
  const auto alignment_started = std::chrono::steady_clock::now();
  matcher.align(aligned_scan, initial_pose.matrix().cast<float>());
  result.gicp_alignment_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - alignment_started).count();
  if (!matcher.hasConverged()) {
    result.status = ScanToMapStatus::kNotConverged;
    return result;
  }

  if (implementation_->target_normals_version != local_map_version) {
    const auto feature_cache_started = std::chrono::steady_clock::now();
    const auto & covariances = matcher.targetCovariances();
    implementation_->target_normals.assign(
      covariances.size(), Eigen::Vector3d::Zero());
    for (std::size_t index = 0U; index < covariances.size(); ++index) {
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
        covariances[index]);
      if (solver.info() == Eigen::Success) {
        implementation_->target_normals[index] = solver.eigenvectors().col(0);
      }
    }
    implementation_->target_normals_version = local_map_version;
    result.target_feature_cache_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - feature_cache_started).count();
  }

  const Eigen::Matrix4d final_matrix =
    matcher.getFinalTransformation().cast<double>();
  if (!final_matrix.allFinite()) {
    result.status = ScanToMapStatus::kInvalidInput;
    return result;
  }
  Eigen::Quaterniond final_rotation(final_matrix.block<3, 3>(0, 0));
  if (!final_rotation.coeffs().allFinite() ||
    final_rotation.squaredNorm() <= std::numeric_limits<double>::epsilon())
  {
    result.status = ScanToMapStatus::kInvalidInput;
    return result;
  }
  final_rotation.normalize();
  result.pose = Eigen::Isometry3d::Identity();
  result.pose.linear() = final_rotation.toRotationMatrix();
  result.pose.translation() = final_matrix.block<3, 1>(0, 3);

  const Eigen::Isometry3d correction = initial_pose.inverse() * result.pose;
  result.correction_translation = correction.translation().norm();
  result.correction_rotation = rotationAngle(correction.rotation());
  result.applied_correction_translation = result.correction_translation;
  result.applied_correction_rotation = result.correction_rotation;
  if (result.correction_translation >
    parameters_.maximum_correction_translation ||
    result.correction_rotation > parameters_.maximum_correction_rotation)
  {
    result.status = ScanToMapStatus::kCorrectionTooLarge;
    return result;
  }

  pcl::search::KdTree<pcl::PointXYZI> search;
  search.setInputCloud(implementation_->target_cloud);
  const double maximum_distance_squared =
    parameters_.maximum_correspondence_distance *
    parameters_.maximum_correspondence_distance;
  double squared_error_sum = 0.0;
  Eigen::Matrix2d translation_information = Eigen::Matrix2d::Zero();
  Eigen::Matrix3d planar_information = Eigen::Matrix3d::Zero();
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  const auto observability_started = std::chrono::steady_clock::now();
  for (const auto & point : aligned_scan) {
    if (search.nearestKSearch(point, 1, indices, squared_distances) == 1 &&
      squared_distances.front() <= maximum_distance_squared)
    {
      ++result.correspondence_count;
      squared_error_sum += squared_distances.front();

      const Eigen::Vector3d & normal = implementation_->target_normals[
        static_cast<std::size_t>(indices.front())];
      if (!normal.allFinite()) {
        continue;
      }
      const Eigen::Vector2d translation_jacobian(normal.x(), normal.y());
      if (translation_jacobian.squaredNorm() <
        kMinimumHorizontalNormalSquared)
      {
        continue;
      }
      const Eigen::Vector3d relative_point(
        static_cast<double>(point.x) - result.pose.translation().x(),
        static_cast<double>(point.y) - result.pose.translation().y(),
        static_cast<double>(point.z) - result.pose.translation().z());
      Eigen::Vector3d planar_jacobian;
      planar_jacobian.head<2>() = translation_jacobian;
      planar_jacobian.z() = normal.dot(
        Eigen::Vector3d::UnitZ().cross(relative_point));
      translation_information +=
        translation_jacobian * translation_jacobian.transpose();
      planar_information += planar_jacobian * planar_jacobian.transpose();
      ++result.observability_correspondences;
    }
  }
  result.observability_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - observability_started).count();
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

  if (result.observability_correspondences > 0U) {
    const double inverse_count =
      1.0 / static_cast<double>(result.observability_correspondences);
    translation_information *= inverse_count;
    planar_information *= inverse_count;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> translation_solver(
      translation_information);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> planar_solver(
      planar_information);
    if (translation_solver.info() == Eigen::Success &&
      planar_solver.info() == Eigen::Success)
    {
      result.translation_information_eigenvalues =
        translation_solver.eigenvalues();
      result.weak_translation_direction =
        translation_solver.eigenvectors().col(0);
      result.planar_information_eigenvalues = planar_solver.eigenvalues();
      const double maximum_translation_information =
        result.translation_information_eigenvalues.maxCoeff();
      if (maximum_translation_information > 0.0) {
        result.translation_information_ratio =
          result.translation_information_eigenvalues.minCoeff() /
          maximum_translation_information;
      }
      result.yaw_information = planar_information(2, 2);
      const double minimum_translation_information =
        result.translation_information_eigenvalues.minCoeff();
      if (maximum_translation_information <
        parameters_.minimum_translation_information)
      {
        result.translation_observable_rank = 0;
      } else if (
        result.translation_information_ratio <
        parameters_.minimum_translation_information_ratio ||
        minimum_translation_information <
        parameters_.minimum_translation_information)
      {
        result.translation_observable_rank = 1;
      } else {
        result.translation_observable_rank = 2;
      }
      result.translation_degenerate = result.translation_observable_rank < 2;
      result.planar_degenerate =
        result.planar_information_eigenvalues.minCoeff() <
        parameters_.minimum_planar_information;
      result.yaw_degenerate =
        result.yaw_information < parameters_.minimum_yaw_information;
      result.degenerate = result.translation_degenerate ||
        result.planar_degenerate || result.yaw_degenerate;
      if (result.translation_observable_rank == 0) {
        result.weak_translation_correction_scale = 0.0;
      } else {
        const double ratio_scale = std::clamp(
          (result.translation_information_ratio -
          parameters_.full_suppression_translation_information_ratio) /
          (parameters_.minimum_translation_information_ratio -
          parameters_.full_suppression_translation_information_ratio),
          0.0, 1.0);
        const double absolute_scale = std::clamp(
          (minimum_translation_information -
          parameters_.full_suppression_translation_information) /
          (parameters_.minimum_translation_information -
          parameters_.full_suppression_translation_information),
          0.0, 1.0);
        result.weak_translation_correction_scale =
          std::min(ratio_scale, absolute_scale);
      }
      if (parameters_.degeneracy_handling_enabled &&
        result.weak_translation_correction_scale < 1.0)
      {
        if (result.translation_observable_rank == 0) {
          result.pose.translation().head<2>() =
            initial_pose.translation().head<2>();
        } else {
          Eigen::Vector2d translation_correction =
            result.pose.translation().head<2>() -
            initial_pose.translation().head<2>();
          const double weak_correction = translation_correction.dot(
            result.weak_translation_direction);
          translation_correction -=
            (1.0 - result.weak_translation_correction_scale) *
            weak_correction * result.weak_translation_direction;
          result.pose.translation().head<2>() =
            initial_pose.translation().head<2>() + translation_correction;
        }
        result.degeneracy_handling_applied = true;
      }
    }
  } else {
    result.weak_translation_correction_scale = 0.0;
    if (parameters_.degeneracy_handling_enabled) {
      result.pose.translation().head<2>() =
        initial_pose.translation().head<2>();
      result.degeneracy_handling_applied = true;
    }
  }

  const Eigen::Isometry3d applied_correction = initial_pose.inverse() * result.pose;
  result.applied_correction_translation = applied_correction.translation().norm();
  result.applied_correction_rotation = rotationAngle(applied_correction.rotation());

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
    parameters_.maximum_correction_rotation <= 0.0 ||
    !std::isfinite(parameters_.minimum_translation_information_ratio) ||
    parameters_.minimum_translation_information_ratio <= 0.0 ||
    parameters_.minimum_translation_information_ratio > 1.0 ||
    !std::isfinite(
      parameters_.full_suppression_translation_information_ratio) ||
    parameters_.full_suppression_translation_information_ratio < 0.0 ||
    parameters_.full_suppression_translation_information_ratio >=
    parameters_.minimum_translation_information_ratio ||
    !std::isfinite(parameters_.minimum_translation_information) ||
    parameters_.minimum_translation_information <= 0.0 ||
    !std::isfinite(parameters_.full_suppression_translation_information) ||
    parameters_.full_suppression_translation_information < 0.0 ||
    parameters_.full_suppression_translation_information >=
    parameters_.minimum_translation_information ||
    !std::isfinite(parameters_.minimum_planar_information) ||
    parameters_.minimum_planar_information <= 0.0 ||
    !std::isfinite(parameters_.minimum_yaw_information) ||
    parameters_.minimum_yaw_information <= 0.0)
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
