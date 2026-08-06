#include "slam_robot_slam_3d/se2_pose_graph_backend.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "slam_robot_slam/pose_graph_2d.hpp"

namespace slam_robot_slam_3d
{
namespace
{

slam_robot_slam::Pose2D toPose2d(const Eigen::Isometry3d & pose)
{
  return {pose.translation().x(), pose.translation().y(),
    std::atan2(pose.rotation()(1, 0), pose.rotation()(0, 0))};
}

Eigen::Isometry3d toPose3d(const slam_robot_slam::Pose2D & pose)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation().head<2>() = Eigen::Vector2d(pose.x, pose.y);
  result.linear() = Eigen::AngleAxisd(
    pose.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return result;
}

}  // namespace

Se2PoseGraphBackend::Se2PoseGraphBackend(Se2PoseGraphBackendParameters parameters)
: parameters_(std::move(parameters))
{
  if (parameters_.maximum_iterations <= 0 ||
    !std::isfinite(parameters_.loop_closure_huber_scale) ||
    parameters_.loop_closure_huber_scale <= 0.0 ||
    !std::isfinite(parameters_.sequential_translation_weight) ||
    !std::isfinite(parameters_.sequential_rotation_weight) ||
    !std::isfinite(parameters_.loop_translation_weight) ||
    !std::isfinite(parameters_.loop_rotation_weight) ||
    parameters_.sequential_translation_weight <= 0.0 ||
    parameters_.sequential_rotation_weight <= 0.0 ||
    parameters_.loop_translation_weight <= 0.0 ||
    parameters_.loop_rotation_weight <= 0.0)
  {
    throw std::invalid_argument("SE(2) pose graph parameters are invalid");
  }
}

Se2PoseGraphBackendResult Se2PoseGraphBackend::optimize(
  const std::vector<GlobalKeyframe> & keyframes,
  const std::vector<Se2LoopConstraint> & loop_constraints) const
{
  Se2PoseGraphBackendResult result;
  result.snapshot_keyframe_count = keyframes.size();
  if (keyframes.empty()) {
    return result;
  }
  slam_robot_slam::PoseGraph2D graph;
  for (std::size_t index = 0U; index < keyframes.size(); ++index) {
    if (keyframes[index].id != index ||
      !keyframes[index].front_end_base_pose.matrix().allFinite())
    {
      throw std::invalid_argument("pose graph keyframe snapshot is invalid");
    }
    graph.addNode(toPose2d(keyframes[index].front_end_base_pose));
    if (index > 0U) {
      const Eigen::Isometry3d relative =
        keyframes[index - 1U].front_end_base_pose.inverse() *
        keyframes[index].front_end_base_pose;
      graph.addConstraint({
          index - 1U, index, toPose2d(relative),
          slam_robot_slam::makeDiagonalPoseGraphInformation(
          parameters_.sequential_translation_weight,
          parameters_.sequential_rotation_weight),
          slam_robot_slam::PoseGraphConstraintType::kSequential});
    }
  }
  for (const auto & loop : loop_constraints) {
    if (loop.source_id >= keyframes.size() || loop.target_id >= keyframes.size() ||
      loop.source_id == loop.target_id || !loop.relative_pose.matrix().allFinite())
    {
      throw std::invalid_argument("pose graph loop constraint is invalid");
    }
    graph.addConstraint({
        loop.source_id, loop.target_id, toPose2d(loop.relative_pose),
        slam_robot_slam::makeDiagonalPoseGraphInformation(
        parameters_.loop_translation_weight, parameters_.loop_rotation_weight),
        slam_robot_slam::PoseGraphConstraintType::kLoopClosure});
  }
  const auto summary = graph.optimize({
      parameters_.maximum_iterations, parameters_.loop_closure_huber_scale});
  result.success = summary.success;
  result.iterations = summary.iterations;
  result.initial_cost = summary.initial_cost;
  result.final_cost = summary.final_cost;
  if (result.success) {
    const Eigen::Isometry3d optimized_latest = toPose3d(graph.nodes().back().pose);
    // The graph is deliberately SE(2).  Project both corrections back to
    // SE(2), rather than allowing roll/pitch from an input odometry sample to
    // leak into the map frame through an Isometry inverse.
    result.map_from_local = toPose3d(toPose2d(
        optimized_latest * keyframes.back().front_end_base_pose.inverse()));
    result.map_from_odom = toPose3d(toPose2d(
        optimized_latest * keyframes.back().odom_base_pose.inverse()));
  }
  return result;
}

}  // namespace slam_robot_slam_3d
