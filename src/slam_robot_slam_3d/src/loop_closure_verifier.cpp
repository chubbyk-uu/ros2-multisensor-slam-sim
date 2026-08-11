#include "slam_robot_slam_3d/loop_closure_verifier.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace slam_robot_slam_3d
{
namespace
{

const GlobalKeyframe * findKeyframe(
  const std::vector<GlobalKeyframe> & keyframes, std::size_t id)
{
  const auto iterator = std::find_if(
    keyframes.begin(), keyframes.end(),
    [id](const auto & keyframe) {return keyframe.id == id;});
  return iterator == keyframes.end() ? nullptr : &*iterator;
}

Eigen::Isometry3d yawRotation(double yaw)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return result;
}

}  // namespace

bool LoopClosureVerificationResult::accepted() const
{
  return status == LoopClosureVerificationStatus::kAccepted;
}

LoopClosureVerifier::LoopClosureVerifier(LoopClosureVerifierParameters parameters)
: parameters_(std::move(parameters)), matcher_(parameters_.matcher)
{
  validateParameters();
}

LoopClosureVerificationResult LoopClosureVerifier::verify(
  const std::vector<GlobalKeyframe> & keyframes,
  std::size_t current_keyframe_id,
  const ScanContextCandidate & candidate)
{
  LoopClosureVerificationResult result;
  result.candidate_keyframe_id = candidate.keyframe_id;
  const GlobalKeyframe * current = findKeyframe(keyframes, current_keyframe_id);
  const GlobalKeyframe * historical = findKeyframe(
    keyframes, candidate.keyframe_id);
  if (current == nullptr || historical == nullptr || !current->registration_scan ||
    !historical->registration_scan)
  {
    return result;
  }

  pcl::PointCloud<pcl::PointXYZI> combined_target;
  const std::size_t first_id = historical->id > parameters_.submap_neighbor_keyframes ?
    historical->id - parameters_.submap_neighbor_keyframes : 0U;
  const std::size_t last_id = historical->id + parameters_.submap_neighbor_keyframes;
  for (const auto & keyframe : keyframes) {
    if (keyframe.id < first_id || keyframe.id > last_id ||
      !keyframe.registration_scan)
    {
      continue;
    }
    pcl::PointCloud<pcl::PointXYZI> transformed;
    const Eigen::Isometry3d sensor_pose =
      keyframe.front_end_base_pose * keyframe.base_to_sensor;
    pcl::transformPointCloud(
      *keyframe.registration_scan, transformed, sensor_pose.matrix().cast<float>());
    combined_target += transformed;
  }
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  const float leaf_size = static_cast<float>(parameters_.submap_voxel_leaf_size);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.setInputCloud(combined_target.makeShared());
  pcl::PointCloud<pcl::PointXYZI> target;
  voxel_filter.filter(target);
  target.is_dense = true;
  result.target_points = target.size();

  const Eigen::Isometry3d historical_sensor_pose =
    historical->front_end_base_pose * historical->base_to_sensor;
  const Eigen::Isometry3d initial_pose =
    historical_sensor_pose * yawRotation(candidate.predicted_yaw);
  const auto registration = matcher_.match(
    *current->registration_scan, target, ++target_version_, initial_pose);
  result.current_sensor_pose = registration.pose;
  result.correspondence_count = registration.correspondence_count;
  result.rmse = registration.rmse;
  result.correction_translation = registration.correction_translation;
  result.correction_rotation = registration.correction_rotation;
  result.degenerate = registration.degenerate;
  const Eigen::Isometry3d front_end_relative =
    historical->front_end_base_pose.inverse() * current->front_end_base_pose;
  const Eigen::Isometry3d verified_base_pose =
    registration.pose * current->base_to_sensor.inverse();
  const Eigen::Isometry3d verified_relative =
    historical->front_end_base_pose.inverse() * verified_base_pose;
  result.front_end_translation_disagreement = (
    front_end_relative.translation() - verified_relative.translation()).norm();
  result.front_end_rotation_disagreement = Eigen::AngleAxisd(
    front_end_relative.rotation().transpose() * verified_relative.rotation()).angle();
  if (!registration.success()) {
    result.status = LoopClosureVerificationStatus::kRegistrationRejected;
    return result;
  }
  result.overlap_ratio = static_cast<double>(result.correspondence_count) /
    static_cast<double>(current->registration_scan->size());
  if (result.overlap_ratio < parameters_.minimum_overlap_ratio) {
    result.status = LoopClosureVerificationStatus::kInsufficientOverlap;
  } else if (registration.degenerate) {
    result.status = LoopClosureVerificationStatus::kDegenerateGeometry;
  } else if (result.front_end_translation_disagreement >
    parameters_.maximum_front_end_translation_disagreement)
  {
    result.status = LoopClosureVerificationStatus::kFrontEndInconsistent;
  } else {
    result.status = LoopClosureVerificationStatus::kAccepted;
  }
  return result;
}

void LoopClosureVerifier::validateParameters() const
{
  if (parameters_.submap_neighbor_keyframes == 0U ||
    !std::isfinite(parameters_.submap_voxel_leaf_size) ||
    parameters_.submap_voxel_leaf_size <= 0.0 ||
    !std::isfinite(parameters_.minimum_overlap_ratio) ||
    parameters_.minimum_overlap_ratio <= 0.0 ||
    parameters_.minimum_overlap_ratio > 1.0 ||
    !std::isfinite(parameters_.maximum_front_end_translation_disagreement) ||
    parameters_.maximum_front_end_translation_disagreement <= 0.0)
  {
    throw std::invalid_argument("loop closure verifier parameters are invalid");
  }
}

const char * toString(LoopClosureVerificationStatus status)
{
  switch (status) {
    case LoopClosureVerificationStatus::kAccepted:
      return "accepted";
    case LoopClosureVerificationStatus::kMissingKeyframe:
      return "missing_keyframe";
    case LoopClosureVerificationStatus::kInsufficientOverlap:
      return "insufficient_overlap";
    case LoopClosureVerificationStatus::kDegenerateGeometry:
      return "degenerate_geometry";
    case LoopClosureVerificationStatus::kFrontEndInconsistent:
      return "front_end_inconsistent";
    case LoopClosureVerificationStatus::kRegistrationRejected:
      return "registration_rejected";
  }
  return "unknown";
}

}  // namespace slam_robot_slam_3d
