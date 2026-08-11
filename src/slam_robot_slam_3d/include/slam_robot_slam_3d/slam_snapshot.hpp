#ifndef SLAM_ROBOT_SLAM_3D__SLAM_SNAPSHOT_HPP_
#define SLAM_ROBOT_SLAM_3D__SLAM_SNAPSHOT_HPP_

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "slam_robot_slam_3d/global_keyframe_map.hpp"
#include "slam_robot_slam_3d/se2_pose_graph_backend.hpp"

namespace slam_robot_slam_3d
{
struct SlamSnapshot
{
  std::vector<GlobalKeyframe> keyframes;
  std::vector<Se2LoopConstraint> loop_constraints;
  std::vector<Eigen::Isometry3d> optimized_base_poses;
  // Where the map frame sat relative to the front end's own frame when this
  // was written. Keyframes store front_end_base_pose in that frame, and the
  // pose graph builds its edges from differences between those poses, so
  // resuming without it would leave restored keyframes in one frame and new
  // ones in another. The first edge across the join would then carry the whole
  // accumulated correction as if it were a measurement.
  Eigen::Isometry3d map_from_local{Eigen::Isometry3d::Identity()};
};

// Versioned binary file.  It is deliberately self-contained: scans remain in
// sensor coordinates, enabling a later build to replay maps without a live TF
// buffer or a sidecar database.
void saveSlamSnapshot(const std::string & path, const SlamSnapshot & snapshot);
SlamSnapshot loadSlamSnapshot(const std::string & path);
}  // namespace slam_robot_slam_3d
#endif
