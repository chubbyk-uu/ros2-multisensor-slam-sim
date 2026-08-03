#ifndef SLAM_ROBOT_SLAM__LOOP_CLOSURE_PROCESSOR_HPP_
#define SLAM_ROBOT_SLAM__LOOP_CLOSURE_PROCESSOR_HPP_

#include <cstddef>
#include <memory>
#include <vector>

#include "slam_robot_slam/correlative_scan_matcher.hpp"
#include "slam_robot_slam/loop_closure_detector.hpp"
#include "slam_robot_slam/pose_graph_2d.hpp"

namespace slam_robot_slam
{

struct LoopClosureKeyframe2D
{
  Pose2D pose;
  double accumulated_distance{0.0};
  std::shared_ptr<const std::vector<Point2D>> points;
  TranslationObservability translation_observability;
};

struct LoopClosureProcessorParameters
{
  LoopClosureCandidateParameters candidate;
  std::size_t candidate_submap_half_width{5U};
  CorrelativeScanMatcherParameters matcher;
  bool reject_degenerate_matches{true};
  std::size_t minimum_candidate_chain_size{10U};
  double maximum_correction_translation{0.50};
  double maximum_correction_rotation{0.25};
  double translation_weight{20.0};
  double rotation_weight{20.0};
  PoseGraphOptimizationOptions optimization;
};

struct LoopClosureProcessingResult
{
  std::size_t current_id{0U};
  std::size_t evaluated_candidates{0U};
  std::size_t rejected_degenerate_candidates{0U};
  bool accepted{false};
  std::size_t candidate_id{0U};
  CorrelativeScanMatcherResult match;
  PoseGraphOptimizationSummary optimization;
  PoseGraphConstraint constraint;
  PoseGraph2D optimized_graph;
};

LoopClosureProcessingResult processLoopClosure(
  const std::vector<LoopClosureKeyframe2D> & keyframes,
  PoseGraph2D graph,
  std::size_t current_id,
  const LoopClosureProcessorParameters & parameters);

}  // namespace slam_robot_slam

#endif  // SLAM_ROBOT_SLAM__LOOP_CLOSURE_PROCESSOR_HPP_
