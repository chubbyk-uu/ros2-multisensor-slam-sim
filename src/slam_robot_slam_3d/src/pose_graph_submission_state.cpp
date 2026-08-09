#include "slam_robot_slam_3d/pose_graph_submission_state.hpp"

#include <algorithm>
#include <stdexcept>

namespace slam_robot_slam_3d
{

PoseGraphSubmissionState::PoseGraphSubmissionState(
  std::size_t minimum_keyframe_interval)
: minimum_keyframe_interval_(minimum_keyframe_interval)
{
  if (minimum_keyframe_interval_ == 0U) {
    throw std::invalid_argument("pose graph minimum keyframe interval must be positive");
  }
}

void PoseGraphSubmissionState::enqueue(
  const std::vector<Se2LoopConstraint> & constraints)
{
  appendUnique(pending_constraints_, constraints);
}

std::optional<PoseGraphSubmission> PoseGraphSubmissionState::begin(
  std::size_t latest_keyframe_id, std::size_t snapshot_keyframe_count)
{
  if (active_submission_.has_value() || pending_constraints_.empty()) {
    return std::nullopt;
  }
  if (has_successful_submission_ &&
    latest_keyframe_id - last_successful_keyframe_id_ <
    minimum_keyframe_interval_)
  {
    return std::nullopt;
  }
  // A failed task waits for a new keyframe before retrying. This prevents a
  // persistent numerical failure from consuming every point-cloud callback.
  if (waiting_for_new_keyframe_after_failure_ && has_last_attempt_ &&
    latest_keyframe_id <= last_attempt_keyframe_id_)
  {
    return std::nullopt;
  }

  PoseGraphSubmission submission;
  submission.task_id = next_task_id_++;
  submission.snapshot_keyframe_count = snapshot_keyframe_count;
  submission.submitted_keyframe_id = latest_keyframe_id;
  submission.constraints = committed_constraints_;
  submission.newly_verified_constraints = std::move(pending_constraints_);
  pending_constraints_.clear();
  appendUnique(submission.constraints, submission.newly_verified_constraints);
  active_submission_ = submission;
  last_attempt_keyframe_id_ = latest_keyframe_id;
  has_last_attempt_ = true;
  waiting_for_new_keyframe_after_failure_ = false;
  return submission;
}

bool PoseGraphSubmissionState::completeSuccess(std::uint64_t task_id)
{
  if (!active_submission_.has_value() || active_submission_->task_id != task_id) {
    return false;
  }
  committed_constraints_ = active_submission_->constraints;
  last_successful_keyframe_id_ = active_submission_->submitted_keyframe_id;
  has_successful_submission_ = true;
  waiting_for_new_keyframe_after_failure_ = false;
  active_submission_.reset();
  return true;
}

bool PoseGraphSubmissionState::completeFailure(std::uint64_t task_id)
{
  if (!active_submission_.has_value() || active_submission_->task_id != task_id) {
    return false;
  }
  std::vector<Se2LoopConstraint> retry_constraints =
    std::move(active_submission_->newly_verified_constraints);
  active_submission_.reset();
  appendUnique(pending_constraints_, retry_constraints);
  waiting_for_new_keyframe_after_failure_ = true;
  return true;
}

const std::vector<Se2LoopConstraint> &
PoseGraphSubmissionState::committedConstraints() const
{
  return committed_constraints_;
}

void PoseGraphSubmissionState::restoreCommitted(
  std::vector<Se2LoopConstraint> constraints,
  const std::size_t latest_keyframe_id)
{
  if (active_submission_.has_value() || !pending_constraints_.empty()) {
    throw std::logic_error("cannot restore a busy pose graph submission state");
  }
  std::vector<Se2LoopConstraint> validated;
  appendUnique(validated, constraints);
  if (validated.size() != constraints.size()) {
    throw std::invalid_argument("restored loop constraints contain duplicates");
  }
  committed_constraints_ = std::move(validated);
  last_successful_keyframe_id_ = latest_keyframe_id;
  has_successful_submission_ = !committed_constraints_.empty();
  waiting_for_new_keyframe_after_failure_ = false;
}

std::size_t PoseGraphSubmissionState::pendingConstraintCount() const
{
  return pending_constraints_.size();
}

bool PoseGraphSubmissionState::taskActive() const
{
  return active_submission_.has_value();
}

void PoseGraphSubmissionState::appendUnique(
  std::vector<Se2LoopConstraint> & destination,
  const std::vector<Se2LoopConstraint> & source)
{
  for (const auto & constraint : source) {
    const auto duplicate = std::find_if(
      destination.begin(), destination.end(),
      [&](const auto & existing) {
        return existing.source_id == constraint.source_id &&
               existing.target_id == constraint.target_id;
      });
    if (duplicate == destination.end()) {
      destination.push_back(constraint);
    }
  }
}

}  // namespace slam_robot_slam_3d
