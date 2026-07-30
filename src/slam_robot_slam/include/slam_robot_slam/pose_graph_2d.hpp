#ifndef SLAM_ROBOT_SLAM__POSE_GRAPH_2D_HPP_
#define SLAM_ROBOT_SLAM__POSE_GRAPH_2D_HPP_

#include <cstddef>
#include <vector>

#include "slam_robot_slam/pose2d.hpp"

namespace slam_robot_slam
{

enum class PoseGraphConstraintType
{
  kSequential,
  kLoopClosure
};

struct PoseGraphNode
{
  std::size_t id;
  Pose2D pose;
};

struct PoseGraphConstraint
{
  std::size_t source_id;
  std::size_t target_id;
  Pose2D relative_pose;
  double translation_weight{20.0};
  double rotation_weight{20.0};
  PoseGraphConstraintType type{PoseGraphConstraintType::kSequential};
};

struct PoseGraphOptimizationOptions
{
  int maximum_iterations{50};
  double loop_closure_huber_scale{1.0};
};

struct PoseGraphOptimizationSummary
{
  bool success{false};
  int iterations{0};
  double initial_cost{0.0};
  double final_cost{0.0};
};

class PoseGraph2D
{
public:
  std::size_t addNode(const Pose2D & initial_pose);
  void removeLastNode();
  std::size_t addConstraint(const PoseGraphConstraint & constraint);
  void removeConstraint(std::size_t constraint_id);

  PoseGraphOptimizationSummary optimize(
    const PoseGraphOptimizationOptions & options = {});

  const std::vector<PoseGraphNode> & nodes() const;
  const std::vector<PoseGraphConstraint> & constraints() const;

private:
  void validateConnectedGraph() const;

  std::vector<PoseGraphNode> nodes_;
  std::vector<PoseGraphConstraint> constraints_;
};

}  // namespace slam_robot_slam

#endif  // SLAM_ROBOT_SLAM__POSE_GRAPH_2D_HPP_
