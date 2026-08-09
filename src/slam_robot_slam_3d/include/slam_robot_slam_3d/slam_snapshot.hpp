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
};

// Versioned binary file.  It is deliberately self-contained: scans remain in
// sensor coordinates, enabling a later build to replay maps without a live TF
// buffer or a sidecar database.
void saveSlamSnapshot(const std::string & path, const SlamSnapshot & snapshot);
SlamSnapshot loadSlamSnapshot(const std::string & path);
}  // namespace slam_robot_slam_3d
#endif
