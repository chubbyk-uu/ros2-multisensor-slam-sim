#ifndef SLAM_ROBOT_SLAM_3D__LOOP_CLOSURE_VERIFIER_HPP_
#define SLAM_ROBOT_SLAM_3D__LOOP_CLOSURE_VERIFIER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "slam_robot_slam_3d/global_keyframe_map.hpp"
#include "slam_robot_slam_3d/scan_context_index.hpp"
#include "slam_robot_slam_3d/scan_to_map_matcher.hpp"

namespace slam_robot_slam_3d
{

struct LoopClosureVerifierParameters
{
  std::size_t submap_neighbor_keyframes{2U};
  double submap_voxel_leaf_size{0.15};
  double minimum_overlap_ratio{0.30};
  double maximum_front_end_translation_disagreement{10.0};
  ScanToMapMatcherParameters matcher;
};

enum class LoopClosureVerificationStatus
{
  kAccepted,
  kMissingKeyframe,
  kInsufficientOverlap,
  kDegenerateGeometry,
  kFrontEndInconsistent,
  kRegistrationRejected,
};

struct LoopClosureVerificationResult
{
  LoopClosureVerificationStatus status{
    LoopClosureVerificationStatus::kMissingKeyframe};
  std::size_t candidate_keyframe_id{0U};
  Eigen::Isometry3d current_sensor_pose{Eigen::Isometry3d::Identity()};
  std::size_t target_points{0U};
  std::size_t correspondence_count{0U};
  double overlap_ratio{0.0};
  double rmse{0.0};
  double correction_translation{0.0};
  double correction_rotation{0.0};
  double front_end_translation_disagreement{0.0};
  double front_end_rotation_disagreement{0.0};
  bool degenerate{true};

  bool accepted() const;
};

class LoopClosureVerifier
{
public:
  explicit LoopClosureVerifier(LoopClosureVerifierParameters parameters);

  LoopClosureVerificationResult verify(
    const std::vector<GlobalKeyframe> & keyframes,
    std::size_t current_keyframe_id,
    const ScanContextCandidate & candidate);

private:
  void validateParameters() const;

  LoopClosureVerifierParameters parameters_;
  ScanToMapMatcher matcher_;
  std::uint64_t target_version_{0U};
};

const char * toString(LoopClosureVerificationStatus status);

}  // namespace slam_robot_slam_3d

#endif  // SLAM_ROBOT_SLAM_3D__LOOP_CLOSURE_VERIFIER_HPP_
