#ifndef SLAM_ROBOT_SLAM__LOOP_CLOSURE_DETECTOR_HPP_
#define SLAM_ROBOT_SLAM__LOOP_CLOSURE_DETECTOR_HPP_

#include <cstddef>
#include <vector>

#include "slam_robot_slam/pose2d.hpp"

namespace slam_robot_slam
{

struct LoopClosureCandidateParameters
{
  std::size_t minimum_keyframe_separation{80U};
  double search_radius{0.8};
  std::size_t maximum_candidates{3U};
};

struct LoopClosureCandidate
{
  std::size_t keyframe_id;
  double distance;
};

std::vector<LoopClosureCandidate> findLoopClosureCandidates(
  const std::vector<Pose2D> & poses,
  std::size_t current_keyframe_id,
  const LoopClosureCandidateParameters & parameters);

}  // namespace slam_robot_slam

#endif  // SLAM_ROBOT_SLAM__LOOP_CLOSURE_DETECTOR_HPP_
