#include "slam_robot_slam/loop_closure_detector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace slam_robot_slam
{
namespace
{

bool isFinitePose(const Pose2D & pose)
{
  return std::isfinite(pose.x) &&
         std::isfinite(pose.y) &&
         std::isfinite(pose.yaw);
}

}  // namespace

std::vector<LoopClosureCandidate> findLoopClosureCandidates(
  const std::vector<Pose2D> & poses,
  const std::size_t current_keyframe_id,
  const LoopClosureCandidateParameters & parameters)
{
  if (parameters.minimum_keyframe_separation == 0U ||
    !std::isfinite(parameters.search_radius) ||
    parameters.search_radius <= 0.0 ||
    parameters.maximum_candidates == 0U)
  {
    throw std::invalid_argument("Invalid loop closure candidate parameters");
  }
  if (current_keyframe_id >= poses.size()) {
    throw std::out_of_range("Current loop closure keyframe does not exist");
  }
  for (const auto & pose : poses) {
    if (!isFinitePose(pose)) {
      throw std::invalid_argument(
              "Loop closure candidate poses must be finite");
    }
  }
  if (current_keyframe_id < parameters.minimum_keyframe_separation) {
    return {};
  }

  const Pose2D & current_pose = poses[current_keyframe_id];
  const std::size_t latest_candidate_id =
    current_keyframe_id - parameters.minimum_keyframe_separation;
  const double squared_search_radius =
    parameters.search_radius * parameters.search_radius;

  std::vector<LoopClosureCandidate> candidates;
  for (std::size_t candidate_id = 0U;
    candidate_id <= latest_candidate_id; ++candidate_id)
  {
    const double delta_x = poses[candidate_id].x - current_pose.x;
    const double delta_y = poses[candidate_id].y - current_pose.y;
    const double squared_distance =
      delta_x * delta_x + delta_y * delta_y;
    if (squared_distance <= squared_search_radius) {
      candidates.push_back(
        LoopClosureCandidate{
          candidate_id,
          std::sqrt(squared_distance)});
    }
  }

  std::sort(
    candidates.begin(),
    candidates.end(),
    [](const LoopClosureCandidate & first,
    const LoopClosureCandidate & second)
    {
      if (first.distance == second.distance) {
        return first.keyframe_id < second.keyframe_id;
      }
      return first.distance < second.distance;
    });
  if (candidates.size() > parameters.maximum_candidates) {
    candidates.resize(parameters.maximum_candidates);
  }
  return candidates;
}

}  // namespace slam_robot_slam
