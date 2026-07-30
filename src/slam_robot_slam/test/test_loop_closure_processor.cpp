#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include "slam_robot_slam/loop_closure_processor.hpp"

namespace slam_robot_slam
{
namespace
{

std::vector<Point2D> makeCornerScan()
{
  std::vector<Point2D> points;
  for (int index = 0; index <= 80; ++index) {
    points.push_back(
      Point2D{
          3.0F,
          static_cast<float>(-2.0 + 0.05 * index)});
  }
  for (int index = 1; index <= 100; ++index) {
    points.push_back(
      Point2D{
          static_cast<float>(3.0 - 0.05 * index),
          2.0F});
  }
  return points;
}

PoseGraph2D makeLoopGraph(const std::vector<Pose2D> & poses)
{
  PoseGraph2D graph;
  for (const auto & pose : poses) {
    graph.addNode(pose);
  }
  for (std::size_t index = 1U; index < poses.size(); ++index) {
    graph.addConstraint(
      PoseGraphConstraint{
          index - 1U,
          index,
          relativePose(poses[index - 1U], poses[index]),
          20.0,
          20.0,
          PoseGraphConstraintType::kSequential});
  }
  return graph;
}

TEST(LoopClosureProcessor, MatchesAndOptimizesGraphCopy)
{
  const std::vector<Pose2D> poses{
    Pose2D{0.0, 0.0, 0.0},
    Pose2D{1.0, 0.0, 0.0},
    Pose2D{1.0, 1.0, 1.57},
    Pose2D{0.0, 1.0, 3.14},
    Pose2D{0.10, 0.0, 0.0}};
  std::vector<LoopClosureKeyframe2D> keyframes;
  double accumulated_distance = 0.0;
  for (std::size_t index = 0U; index < poses.size(); ++index) {
    if (index > 0U) {
      accumulated_distance += std::hypot(
        poses[index].x - poses[index - 1U].x,
        poses[index].y - poses[index - 1U].y);
    }
    keyframes.push_back(
      LoopClosureKeyframe2D{
          poses[index],
          accumulated_distance,
          std::make_shared<const std::vector<Point2D>>(makeCornerScan())});
  }
  PoseGraph2D graph = makeLoopGraph(poses);
  const std::size_t original_constraint_count = graph.constraints().size();

  LoopClosureProcessorParameters parameters;
  parameters.candidate.minimum_keyframe_separation = 2U;
  parameters.candidate.minimum_travel_distance = 2.0;
  parameters.candidate.search_radius = 0.3;
  parameters.candidate.maximum_candidates = 2U;
  parameters.candidate_submap_half_width = 0U;
  parameters.minimum_candidate_chain_size = 1U;
  parameters.matcher.grid_resolution = 0.02;
  parameters.matcher.smear_deviation = 0.04;
  parameters.matcher.linear_search_window = 0.20;
  parameters.matcher.angular_search_window = 0.10;
  parameters.matcher.coarse_linear_resolution = 0.02;
  parameters.matcher.coarse_angular_resolution = 0.02;
  parameters.matcher.fine_linear_resolution = 0.005;
  parameters.matcher.fine_angular_resolution = 0.005;
  parameters.matcher.minimum_score = 0.5;
  parameters.matcher.minimum_matched_points = 80U;

  const auto result =
    processLoopClosure(keyframes, graph, 4U, parameters);

  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(result.candidate_id, 0U);
  EXPECT_GT(result.match.score, 0.8);
  EXPECT_EQ(
    result.optimized_graph.constraints().size(),
    original_constraint_count + 1U);
  EXPECT_EQ(graph.constraints().size(), original_constraint_count);
}

TEST(LoopClosureProcessor, RejectsMismatchedState)
{
  PoseGraph2D graph;
  graph.addNode(Pose2D{});

  EXPECT_THROW(
    processLoopClosure(
      std::vector<LoopClosureKeyframe2D>{},
      graph,
      0U,
      LoopClosureProcessorParameters{}),
    std::invalid_argument);
}

TEST(LoopClosureProcessor, RejectsImplausiblyLargeCorrection)
{
  const std::vector<Pose2D> poses{
    Pose2D{0.0, 0.0, 0.0},
    Pose2D{1.0, 0.0, 0.0},
    Pose2D{0.15, 0.0, 0.0}};
  std::vector<LoopClosureKeyframe2D> keyframes;
  for (std::size_t index = 0U; index < poses.size(); ++index) {
    keyframes.push_back(
      LoopClosureKeyframe2D{
          poses[index],
          static_cast<double>(index),
          std::make_shared<const std::vector<Point2D>>(makeCornerScan())});
  }
  LoopClosureProcessorParameters parameters;
  parameters.candidate.minimum_keyframe_separation = 2U;
  parameters.candidate.minimum_travel_distance = 2.0;
  parameters.candidate.search_radius = 0.3;
  parameters.candidate_submap_half_width = 0U;
  parameters.minimum_candidate_chain_size = 1U;
  parameters.maximum_correction_translation = 0.05;
  parameters.matcher.linear_search_window = 0.2;
  parameters.matcher.minimum_score = 0.5;
  parameters.matcher.minimum_matched_points = 80U;

  const auto result = processLoopClosure(
    keyframes,
    makeLoopGraph(poses),
    2U,
    parameters);

  EXPECT_FALSE(result.accepted);
}

}  // namespace
}  // namespace slam_robot_slam
