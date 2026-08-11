#ifndef SLAM_ROBOT_SLAM_3D__GLOBAL_MAP_POSE_SELECTOR_HPP_
#define SLAM_ROBOT_SLAM_3D__GLOBAL_MAP_POSE_SELECTOR_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Geometry>

#include "slam_robot_slam_3d/global_keyframe_map.hpp"

namespace slam_robot_slam_3d
{

// Chooses the pose each keyframe is replayed from when the map is rebuilt.
//
// An optimisation owns a prefix of the trajectory and publishes poses for it
// directly. Keyframes added since carry the pose the front end scan-matched for
// them, which only needs moving into the map frame by the same correction that
// optimisation produced.
//
// The alternative, placing them from wheel odometry, discards a scan-matched
// estimate that is already stored beside it. That mattered more than it looks:
// the correction only changes when an optimisation commits, a commit needs an
// accepted loop closure, and an exploration pass often closes none. A measured
// run then drew its entire map from raw odometry and stretched 2.6 m past the
// world's walls while the front end was tracking ground truth to 0.031 m.
inline std::vector<Eigen::Isometry3d> selectGlobalMapPoses(
  const std::vector<GlobalKeyframe> & keyframes,
  const std::vector<Eigen::Isometry3d> & optimized_base_poses,
  const Eigen::Isometry3d & map_from_local)
{
  std::vector<Eigen::Isometry3d> poses;
  poses.reserve(keyframes.size());
  for (std::size_t index = 0U; index < keyframes.size(); ++index) {
    poses.push_back(index < optimized_base_poses.size() ?
      optimized_base_poses[index] :
      map_from_local * keyframes[index].front_end_base_pose);
  }
  return poses;
}

// Brings restored keyframes into the frame mapping will resume in.
//
// Keyframes store front_end_base_pose in the front end's own frame, while a
// resumed session creates new ones in the map frame. Left as they are, the
// pose graph's first edge across that join would carry the whole accumulated
// correction as if it were a measurement.
//
// A rigid transform applied to every pose leaves all relative poses unchanged,
// and relative poses are what the edges are built from. Rebasing onto the
// optimised poses instead would not: optimisation changes the relative poses
// the graph is later asked to re-derive, so those corrections would be counted
// a second time.
inline void rebaseFrontEndPoses(
  std::vector<GlobalKeyframe> & keyframes,
  const Eigen::Isometry3d & map_from_local)
{
  for (auto & keyframe : keyframes) {
    keyframe.front_end_base_pose = map_from_local * keyframe.front_end_base_pose;
  }
}

}  // namespace slam_robot_slam_3d

#endif  // SLAM_ROBOT_SLAM_3D__GLOBAL_MAP_POSE_SELECTOR_HPP_
