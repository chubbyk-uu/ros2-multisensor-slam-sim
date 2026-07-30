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
    if (!keyframe.points ||
      !std::isfinite(keyframe.accumulated_distance) ||
      keyframe.accumulated_distance < 0.0)
    {
      throw std::invalid_argument(
              "Loop closure keyframes must contain valid distance and points");
    }
  }
  if (!std::isfinite(parameters.translation_weight) ||
    !std::isfinite(parameters.rotation_weight) ||
    parameters.translation_weight <= 0.0 ||
    parameters.rotation_weight <= 0.0 ||
    parameters.minimum_candidate_chain_size == 0U ||
    !std::isfinite(parameters.maximum_correction_translation) ||
    !std::isfinite(parameters.maximum_correction_rotation) ||
    parameters.maximum_correction_translation <= 0.0 ||
    parameters.maximum_correction_rotation <= 0.0)
  {
    throw std::invalid_argument("Loop closure weights must be finite and positive");
  }
  validateCorrelativeScanMatcherParameters(parameters.matcher);
}

std::size_t candidateChainSize(
  const std::vector<Pose2D> & poses,
  const std::size_t candidate_id,
  const std::size_t latest_historical_id,
  const double search_radius)
{
  const auto is_near_candidate =
    [&poses, candidate_id, search_radius](const std::size_t index) {
      return std::hypot(
        poses[index].x - poses[candidate_id].x,
        poses[index].y - poses[candidate_id].y) <= search_radius;
    };
  std::size_t chain_size = 1U;
  for (std::size_t index = candidate_id;
    index > 0U && is_near_candidate(index - 1U); --index)
  {
    ++chain_size;
  }
  for (std::size_t index = candidate_id + 1U;
    index <= latest_historical_id && is_near_candidate(index); ++index)
  {
    ++chain_size;
  }
  return chain_size;
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
  std::vector<double> accumulated_distances;
  graph_poses.reserve(graph.nodes().size());
  accumulated_distances.reserve(keyframes.size());
  for (std::size_t index = 0U; index < graph.nodes().size(); ++index) {
    graph_poses.push_back(graph.nodes()[index].pose);
    accumulated_distances.push_back(
      keyframes[index].accumulated_distance);
  }
  const auto candidates = findLoopClosureCandidates(
    graph_poses,
    accumulated_distances,
    current_id,
    parameters.candidate);
  result.evaluated_candidates = candidates.size();
  if (candidates.empty()) {
    return result;
  }

  bool found_match = false;
  const std::size_t latest_historical_id =
    current_id - parameters.candidate.minimum_keyframe_separation;
  for (const auto & candidate : candidates) {
    if (candidateChainSize(
        graph_poses,
        candidate.keyframe_id,
        latest_historical_id,
        parameters.candidate.search_radius) <
      parameters.minimum_candidate_chain_size)
    {
      continue;
    }
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
    const Pose2D correction =
      relativePose(graph_poses[current_id], match.pose);
    if (match.success &&
      std::hypot(correction.x, correction.y) <=
      parameters.maximum_correction_translation &&
      std::abs(correction.yaw) <=
      parameters.maximum_correction_rotation &&
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
