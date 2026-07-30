#include "slam_robot_slam/pose_graph_2d.hpp"

#include <ceres/ceres.h>

#include <cmath>
#include <deque>
#include <stdexcept>
#include <vector>

namespace slam_robot_slam
{
namespace
{

template<typename T>
T normalizeAngleForCeres(const T & angle)
{
  return ceres::atan2(ceres::sin(angle), ceres::cos(angle));
}

class AngleManifold
{
public:
  template<typename T>
  bool Plus(
    const T * angle,
    const T * delta,
    T * result) const
  {
    result[0] = normalizeAngleForCeres(angle[0] + delta[0]);
    return true;
  }

  template<typename T>
  bool Minus(
    const T * first,
    const T * second,
    T * result) const
  {
    result[0] = normalizeAngleForCeres(first[0] - second[0]);
    return true;
  }
};

class PoseGraphErrorTerm
{
public:
  explicit PoseGraphErrorTerm(const PoseGraphConstraint & constraint)
  : measurement_(constraint.relative_pose),
    translation_weight_(constraint.translation_weight),
    rotation_weight_(constraint.rotation_weight)
  {
  }

  template<typename T>
  bool operator()(
    const T * const source_x,
    const T * const source_y,
    const T * const source_yaw,
    const T * const target_x,
    const T * const target_y,
    const T * const target_yaw,
    T * residuals) const
  {
    const T delta_x = target_x[0] - source_x[0];
    const T delta_y = target_y[0] - source_y[0];
    const T cosine = ceres::cos(source_yaw[0]);
    const T sine = ceres::sin(source_yaw[0]);

    const T predicted_x = cosine * delta_x + sine * delta_y;
    const T predicted_y = -sine * delta_x + cosine * delta_y;
    residuals[0] =
      T(translation_weight_) * (predicted_x - T(measurement_.x));
    residuals[1] =
      T(translation_weight_) * (predicted_y - T(measurement_.y));
    residuals[2] = T(rotation_weight_) * normalizeAngleForCeres(
      (target_yaw[0] - source_yaw[0]) - T(measurement_.yaw));
    return true;
  }

  static ceres::CostFunction * create(
    const PoseGraphConstraint & constraint)
  {
    return new ceres::AutoDiffCostFunction<
      PoseGraphErrorTerm, 3, 1, 1, 1, 1, 1, 1>(
      new PoseGraphErrorTerm(constraint));
  }

private:
  Pose2D measurement_;
  double translation_weight_;
  double rotation_weight_;
};

struct OptimizedPose
{
  double x;
  double y;
  double yaw;
};

}  // namespace

std::size_t PoseGraph2D::addNode(const Pose2D & initial_pose)
{
  if (!isFinitePose(initial_pose)) {
    throw std::invalid_argument("Pose graph node must contain a finite pose");
  }
  const std::size_t id = nodes_.size();
  nodes_.push_back(
    PoseGraphNode{
      id,
      Pose2D{
        initial_pose.x,
        initial_pose.y,
        normalizeAngle(initial_pose.yaw)}});
  return id;
}

void PoseGraph2D::removeLastNode()
{
  if (nodes_.empty()) {
    throw std::logic_error("Cannot remove a node from an empty pose graph");
  }
  const std::size_t node_id = nodes_.back().id;
  for (const auto & constraint : constraints_) {
    if (constraint.source_id == node_id ||
      constraint.target_id == node_id)
    {
      throw std::logic_error(
              "Cannot remove a pose graph node referenced by a constraint");
    }
  }
  nodes_.pop_back();
}

std::size_t PoseGraph2D::addConstraint(
  const PoseGraphConstraint & constraint)
{
  if (constraint.source_id >= nodes_.size() ||
    constraint.target_id >= nodes_.size() ||
    constraint.source_id == constraint.target_id ||
    !isFinitePose(constraint.relative_pose) ||
    !std::isfinite(constraint.translation_weight) ||
    !std::isfinite(constraint.rotation_weight) ||
    constraint.translation_weight <= 0.0 ||
    constraint.rotation_weight <= 0.0)
  {
    throw std::invalid_argument("Invalid pose graph constraint");
  }
  const std::size_t constraint_id = constraints_.size();
  constraints_.push_back(constraint);
  return constraint_id;
}

void PoseGraph2D::removeConstraint(const std::size_t constraint_id)
{
  if (constraint_id >= constraints_.size()) {
    throw std::out_of_range("Pose graph constraint does not exist");
  }
  constraints_.erase(
    constraints_.begin() +
    static_cast<std::ptrdiff_t>(constraint_id));
}

void PoseGraph2D::setNodePoses(const std::vector<Pose2D> & poses)
{
  if (poses.size() != nodes_.size()) {
    throw std::invalid_argument(
            "Pose graph update must contain one pose per node");
  }
  for (const auto & pose : poses) {
    if (!isFinitePose(pose)) {
      throw std::invalid_argument(
              "Pose graph update must contain only finite poses");
    }
  }
  for (std::size_t index = 0U; index < poses.size(); ++index) {
    nodes_[index].pose = Pose2D{
      poses[index].x,
      poses[index].y,
      normalizeAngle(poses[index].yaw)};
  }
}

void PoseGraph2D::validateConnectedGraph() const
{
  if (nodes_.size() < 2U) {
    return;
  }

  std::vector<std::vector<std::size_t>> adjacency(nodes_.size());
  for (const auto & constraint : constraints_) {
    adjacency[constraint.source_id].push_back(constraint.target_id);
    adjacency[constraint.target_id].push_back(constraint.source_id);
  }

  std::vector<bool> visited(nodes_.size(), false);
  std::deque<std::size_t> pending{0U};
  visited[0] = true;
  while (!pending.empty()) {
    const std::size_t node = pending.front();
    pending.pop_front();
    for (const std::size_t neighbor : adjacency[node]) {
      if (!visited[neighbor]) {
        visited[neighbor] = true;
        pending.push_back(neighbor);
      }
    }
  }
  for (const bool node_was_visited : visited) {
    if (!node_was_visited) {
      throw std::invalid_argument("Pose graph must be connected");
    }
  }
}

PoseGraphOptimizationSummary PoseGraph2D::optimize(
  const PoseGraphOptimizationOptions & options)
{
  if (options.maximum_iterations < 1 ||
    !std::isfinite(options.loop_closure_huber_scale) ||
    options.loop_closure_huber_scale <= 0.0)
  {
    throw std::invalid_argument("Invalid pose graph optimization options");
  }
  validateConnectedGraph();

  PoseGraphOptimizationSummary result;
  if (nodes_.size() <= 1U) {
    result.success = true;
    return result;
  }

  std::vector<OptimizedPose> poses;
  poses.reserve(nodes_.size());
  for (const auto & node : nodes_) {
    poses.push_back(
      OptimizedPose{node.pose.x, node.pose.y, node.pose.yaw});
  }

  ceres::Problem problem;
  auto * angle_manifold =
    new ceres::AutoDiffManifold<AngleManifold, 1, 1>;
  for (auto & pose : poses) {
    problem.AddParameterBlock(&pose.x, 1);
    problem.AddParameterBlock(&pose.y, 1);
    problem.AddParameterBlock(&pose.yaw, 1, angle_manifold);
  }

  for (const auto & constraint : constraints_) {
    auto & source = poses[constraint.source_id];
    auto & target = poses[constraint.target_id];
    ceres::LossFunction * loss = nullptr;
    if (constraint.type == PoseGraphConstraintType::kLoopClosure) {
      loss = new ceres::HuberLoss(options.loop_closure_huber_scale);
    }
    problem.AddResidualBlock(
      PoseGraphErrorTerm::create(constraint),
      loss,
      &source.x,
      &source.y,
      &source.yaw,
      &target.x,
      &target.y,
      &target.yaw);
  }

  problem.SetParameterBlockConstant(&poses.front().x);
  problem.SetParameterBlockConstant(&poses.front().y);
  problem.SetParameterBlockConstant(&poses.front().yaw);

  ceres::Solver::Options solver_options;
  solver_options.max_num_iterations = options.maximum_iterations;
  solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  solver_options.num_threads = 1;
  solver_options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);
  result.success = summary.IsSolutionUsable();
  result.iterations = static_cast<int>(summary.iterations.size());
  result.initial_cost = summary.initial_cost;
  result.final_cost = summary.final_cost;
  if (!result.success) {
    return result;
  }

  for (std::size_t index = 0U; index < nodes_.size(); ++index) {
    nodes_[index].pose = Pose2D{
      poses[index].x,
      poses[index].y,
      normalizeAngle(poses[index].yaw)};
  }
  return result;
}

const std::vector<PoseGraphNode> & PoseGraph2D::nodes() const
{
  return nodes_;
}

const std::vector<PoseGraphConstraint> & PoseGraph2D::constraints() const
{
  return constraints_;
}

}  // namespace slam_robot_slam
