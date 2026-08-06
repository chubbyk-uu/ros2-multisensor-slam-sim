#ifndef SLAM_ROBOT_SLAM_3D__POSE_GRAPH_SUBMISSION_STATE_HPP_
#define SLAM_ROBOT_SLAM_3D__POSE_GRAPH_SUBMISSION_STATE_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "slam_robot_slam_3d/se2_pose_graph_backend.hpp"

namespace slam_robot_slam_3d
{

// Keeps graph constraints transactional across asynchronous optimization. A
// failed task returns only its newly verified edges to pending state; already
// committed edges are never duplicated or discarded.
struct PoseGraphSubmission
{
  std::uint64_t task_id{0U};
  std::size_t snapshot_keyframe_count{0U};
  std::size_t submitted_keyframe_id{0U};
  std::vector<Se2LoopConstraint> constraints;
  std::vector<Se2LoopConstraint> newly_verified_constraints;
};

class PoseGraphSubmissionState
{
public:
  explicit PoseGraphSubmissionState(std::size_t minimum_keyframe_interval);

  void enqueue(const std::vector<Se2LoopConstraint> & constraints);
  std::optional<PoseGraphSubmission> begin(
    std::size_t latest_keyframe_id, std::size_t snapshot_keyframe_count);
  bool completeSuccess(std::uint64_t task_id);
  bool completeFailure(std::uint64_t task_id);

  const std::vector<Se2LoopConstraint> & committedConstraints() const;
  std::size_t pendingConstraintCount() const;
  bool taskActive() const;

private:
  static void appendUnique(
    std::vector<Se2LoopConstraint> & destination,
    const std::vector<Se2LoopConstraint> & source);

  std::size_t minimum_keyframe_interval_{0U};
  std::vector<Se2LoopConstraint> committed_constraints_;
  std::vector<Se2LoopConstraint> pending_constraints_;
  std::optional<PoseGraphSubmission> active_submission_;
  std::uint64_t next_task_id_{1U};
  std::size_t last_attempt_keyframe_id_{0U};
  bool has_last_attempt_{false};
  bool waiting_for_new_keyframe_after_failure_{false};
  std::size_t last_successful_keyframe_id_{0U};
  bool has_successful_submission_{false};
};

}  // namespace slam_robot_slam_3d

#endif  // SLAM_ROBOT_SLAM_3D__POSE_GRAPH_SUBMISSION_STATE_HPP_
