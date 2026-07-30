#include "slam_robot_slam/loop_closure_processor.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <utility>
#include <vector>

namespace slam_robot_slam
{
namespace
{

void validateInputs(
  const std::vector<LoopClosureKeyframe2D> & keyframes,
  const PoseGraph2D & graph,
  const std::size_t current_id,
  const LoopClosureProcessorParameters & parameters)
{
  if (keyframes.size() != graph.nodes().size()) {
    throw std::invalid_argument(
            "Loop closure keyframes and graph nodes must have equal sizes");
  }
  if (current_id >= keyframes.size()) {
    throw std::out_of_range("Loop closure current keyframe does not exist");
  }
  for (const auto & keyframe : keyframes) {
    if (!keyframe.points) {
      throw std::invalid_argument(
              "Loop closure keyframe point storage must not be null");
    }
  }
  if (!std::isfinite(parameters.translation_weight) ||
    !std::isfinite(parameters.rotation_weight) ||
    parameters.translation_weight <= 0.0 ||
    parameters.rotation_weight <= 0.0)
  {
    throw std::invalid_argument("Loop closure weights must be finite and positive");
  }
  validateCorrelativeScanMatcherParameters(parameters.matcher);
}

std::vector<Point2D> buildReferencePoints(
  const std::vector<LoopClosureKeyframe2D> & keyframes,
  const std::vector<Pose2D> & graph_poses,
  const std::size_t candidate_id,
  const std::size_t current_id,
  const LoopClosureProcessorParameters & parameters)
{
  const std::size_t latest_historical_id =
    current_id - parameters.candidate.minimum_keyframe_separation;
  const std::size_t first_id =
    candidate_id > parameters.candidate_submap_half_width ?
    candidate_id - parameters.candidate_submap_half_width : 0U;
  const std::size_t last_id = std::min(
    candidate_id + parameters.candidate_submap_half_width,
    latest_historical_id);

  std::size_t total_points = 0U;
  for (std::size_t index = first_id; index <= last_id; ++index) {
    total_points += keyframes[index].points->size();
  }

  std::vector<Point2D> reference_points;
  reference_points.reserve(total_points);
  for (std::size_t index = first_id; index <= last_id; ++index) {
    const auto & keyframe = keyframes[index];
    std::transform(
      keyframe.points->begin(),
      keyframe.points->end(),
      std::back_inserter(reference_points),
      [&graph_poses, index](const Point2D & point) {
        return transformPoint(graph_poses[index], point);
      });
  }
  return reference_points;
}

}  // namespace

LoopClosureProcessingResult processLoopClosure(
  const std::vector<LoopClosureKeyframe2D> & keyframes,
  PoseGraph2D graph,
  const std::size_t current_id,
  const LoopClosureProcessorParameters & parameters)
{
  validateInputs(keyframes, graph, current_id, parameters);

  LoopClosureProcessingResult result;
  result.current_id = current_id;
  result.optimized_graph = graph;

  std::vector<Pose2D> graph_poses;
  graph_poses.reserve(graph.nodes().size());
  for (const auto & node : graph.nodes()) {
    graph_poses.push_back(node.pose);
  }
  const auto candidates = findLoopClosureCandidates(
    graph_poses, current_id, parameters.candidate);
  result.evaluated_candidates = candidates.size();
  if (candidates.empty()) {
    return result;
  }

  bool found_match = false;
  for (const auto & candidate : candidates) {
    const auto reference_points = buildReferencePoints(
      keyframes,
      graph_poses,
      candidate.keyframe_id,
      current_id,
      parameters);
    const auto match = matchCorrelative(
      reference_points,
      *keyframes[current_id].points,
      graph_poses[current_id],
      parameters.matcher);
    if (match.success &&
      (!found_match || match.score > result.match.score))
    {
      found_match = true;
      result.candidate_id = candidate.keyframe_id;
      result.match = match;
    }
  }
  if (!found_match) {
    return result;
  }

  result.constraint = PoseGraphConstraint{
    result.candidate_id,
    current_id,
    relativePose(graph_poses[result.candidate_id], result.match.pose),
    parameters.translation_weight,
    parameters.rotation_weight,
    PoseGraphConstraintType::kLoopClosure};
  graph.addConstraint(result.constraint);
  result.optimization = graph.optimize(parameters.optimization);
  if (!result.optimization.success) {
    return result;
  }

  result.accepted = true;
  result.optimized_graph = std::move(graph);
  return result;
}

}  // namespace slam_robot_slam
