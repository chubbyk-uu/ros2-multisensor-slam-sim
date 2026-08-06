#ifndef SLAM_ROBOT_SLAM_3D__SE2_POSE_GRAPH_BACKEND_HPP_
#define SLAM_ROBOT_SLAM_3D__SE2_POSE_GRAPH_BACKEND_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Geometry>

#include "slam_robot_slam_3d/global_keyframe_map.hpp"

namespace slam_robot_slam_3d
{

struct Se2LoopConstraint
{
  std::size_t source_id{0U};
  std::size_t target_id{0U};
  Eigen::Isometry3d relative_pose{Eigen::Isometry3d::Identity()};
};

struct Se2PoseGraphBackendParameters
{
  int maximum_iterations{50};
  double loop_closure_huber_scale{1.0};
  double sequential_translation_weight{20.0};
  double sequential_rotation_weight{20.0};
  double loop_translation_weight{40.0};
  double loop_rotation_weight{40.0};
};

struct Se2PoseGraphBackendResult
{
  bool success{false};
  std::size_t snapshot_keyframe_count{0U};
  int iterations{0};
  double initial_cost{0.0};
  double final_cost{0.0};
  Eigen::Isometry3d map_from_local{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d map_from_odom{Eigen::Isometry3d::Identity()};
};

class Se2PoseGraphBackend
{
public:
  explicit Se2PoseGraphBackend(Se2PoseGraphBackendParameters parameters);

  Se2PoseGraphBackendResult optimize(
    const std::vector<GlobalKeyframe> & keyframes,
    const std::vector<Se2LoopConstraint> & loop_constraints) const;

private:
  Se2PoseGraphBackendParameters parameters_;
};

}  // namespace slam_robot_slam_3d

#endif  // SLAM_ROBOT_SLAM_3D__SE2_POSE_GRAPH_BACKEND_HPP_
