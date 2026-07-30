#include "slam_robot_slam/loop_closure_detector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace slam_robot_slam
{
std::vector<LoopClosureCandidate> findLoopClosureCandidates(
  const std::vector<Pose2D> & poses,
  const std::vector<double> & accumulated_distances,
  const std::size_t current_keyframe_id,
  const LoopClosureCandidateParameters & parameters)
{
  if (parameters.minimum_keyframe_separation == 0U ||
    !std::isfinite(parameters.minimum_travel_distance) ||
    parameters.minimum_travel_distance <= 0.0 ||
    !std::isfinite(parameters.search_radius) ||
    parameters.search_radius <= 0.0 ||
    parameters.maximum_candidates == 0U)
  {
    throw std::invalid_argument("Invalid loop closure candidate parameters");
  }
  if (current_keyframe_id >= poses.size()) {
    throw std::out_of_range("Current loop closure keyframe does not exist");
  }
  if (accumulated_distances.size() != poses.size()) {
    throw std::invalid_argument(
            "Loop closure distances must match pose count");
  }
  double previous_distance = 0.0;
  for (std::size_t index = 0U;
    index < accumulated_distances.size(); ++index)
  {
    const double distance = accumulated_distances[index];
    if (!std::isfinite(distance) || distance < 0.0 ||
      (index > 0U && distance < previous_distance))
    {
      throw std::invalid_argument(
              "Loop closure distances must be finite and nondecreasing");
    }
    previous_distance = distance;
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
    const double travel_separation =
      accumulated_distances[current_keyframe_id] -
      accumulated_distances[candidate_id];
    if (squared_distance <= squared_search_radius &&
      travel_separation >= parameters.minimum_travel_distance)
    {
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
