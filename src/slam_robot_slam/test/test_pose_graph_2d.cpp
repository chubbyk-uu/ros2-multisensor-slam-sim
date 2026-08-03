#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "slam_robot_slam/pose_graph_2d.hpp"

namespace slam_robot_slam
{
namespace
{

PoseGraphConstraint sequentialConstraint(
  const std::size_t source,
  const std::size_t target,
  const Pose2D & relative_pose)
{
  return PoseGraphConstraint{
    source,
    target,
    relative_pose,
    makeDiagonalPoseGraphInformation(20.0, 20.0),
    PoseGraphConstraintType::kSequential};
}

TEST(PoseGraph2D, KeepsConsistentSequentialChain)
{
  PoseGraph2D graph;
  graph.addNode(Pose2D{0.0, 0.0, 0.0});
  graph.addNode(Pose2D{1.0, 0.0, 0.1});
  graph.addNode(Pose2D{1.995, 0.0998, 0.1});
  graph.addConstraint(
    sequentialConstraint(0U, 1U, Pose2D{1.0, 0.0, 0.1}));
  graph.addConstraint(
    sequentialConstraint(1U, 2U, Pose2D{1.0, 0.0, 0.0}));

  const auto result = graph.optimize();

  ASSERT_TRUE(result.success);
  EXPECT_NEAR(graph.nodes()[2].pose.x, 1.995, 1.0e-3);
  EXPECT_NEAR(graph.nodes()[2].pose.y, 0.0998, 1.0e-3);
  EXPECT_LT(result.final_cost, 1.0e-6);
}

TEST(PoseGraph2D, LoopClosureReducesAccumulatedDrift)
{
  PoseGraph2D graph;
  graph.addNode(Pose2D{0.0, 0.0, 0.0});
  graph.addNode(Pose2D{1.1, 0.0, 0.0});
  graph.addNode(Pose2D{2.2, 0.0, 0.0});
  graph.addNode(Pose2D{3.3, 0.0, 0.0});
  graph.addConstraint(
    sequentialConstraint(0U, 1U, Pose2D{1.1, 0.0, 0.0}));
  graph.addConstraint(
    sequentialConstraint(1U, 2U, Pose2D{1.1, 0.0, 0.0}));
  graph.addConstraint(
    sequentialConstraint(2U, 3U, Pose2D{1.1, 0.0, 0.0}));
  graph.addConstraint(
    PoseGraphConstraint{
        3U,
        0U,
        Pose2D{-3.0, 0.0, 0.0},
        makeDiagonalPoseGraphInformation(30.0, 30.0),
        PoseGraphConstraintType::kLoopClosure});

  const double error_before =
    std::abs(relativePose(
      graph.nodes()[3].pose,
      graph.nodes()[0].pose).x + 3.0);
  PoseGraphOptimizationOptions options;
  options.loop_closure_huber_scale = 10.0;
  const auto result = graph.optimize(options);
  const double error_after =
    std::abs(relativePose(
      graph.nodes()[3].pose,
      graph.nodes()[0].pose).x + 3.0);

  ASSERT_TRUE(result.success);
  EXPECT_LT(result.final_cost, result.initial_cost);
  EXPECT_LT(error_after, error_before);
  EXPECT_NEAR(graph.nodes().front().pose.x, 0.0, 1.0e-12);
}

TEST(PoseGraph2D, HandlesAngleWrapAround)
{
  PoseGraph2D graph;
  graph.addNode(Pose2D{0.0, 0.0, 3.13});
  graph.addNode(Pose2D{-1.0, 0.0, -3.12});
  const Pose2D measurement =
    relativePose(graph.nodes()[0].pose, graph.nodes()[1].pose);
  graph.addConstraint(sequentialConstraint(0U, 1U, measurement));

  const auto result = graph.optimize();

  ASSERT_TRUE(result.success);
  EXPECT_NEAR(
    normalizeAngle(
      graph.nodes()[1].pose.yaw -
      graph.nodes()[0].pose.yaw),
    measurement.yaw,
    1.0e-6);
}

TEST(PoseGraph2D, RobustLoopKernelLimitsOutlierInfluence)
{
  PoseGraph2D graph;
  graph.addNode(Pose2D{0.0, 0.0, 0.0});
  graph.addNode(Pose2D{1.0, 0.0, 0.0});
  graph.addNode(Pose2D{2.0, 0.0, 0.0});
  graph.addConstraint(
    sequentialConstraint(0U, 1U, Pose2D{1.0, 0.0, 0.0}));
  graph.addConstraint(
    sequentialConstraint(1U, 2U, Pose2D{1.0, 0.0, 0.0}));
  graph.addConstraint(
    PoseGraphConstraint{
        2U,
        0U,
        Pose2D{-20.0, 0.0, 0.0},
        makeDiagonalPoseGraphInformation(20.0, 20.0),
        PoseGraphConstraintType::kLoopClosure});

  PoseGraphOptimizationOptions options;
  options.loop_closure_huber_scale = 0.5;
  const auto result = graph.optimize(options);

  ASSERT_TRUE(result.success);
  EXPECT_NEAR(graph.nodes()[2].pose.x, 2.0, 0.1);
}

TEST(PoseGraph2D, HonorsAnisotropicTranslationInformation)
{
  PoseGraph2D graph;
  graph.addNode(Pose2D{0.0, 0.0, 0.0});
  graph.addNode(Pose2D{1.0, 1.0, 0.0});
  graph.addConstraint(
    sequentialConstraint(0U, 1U, Pose2D{1.0, 1.0, 0.0}));
  graph.addConstraint(
    PoseGraphConstraint{
        0U,
        1U,
        Pose2D{1.0, 0.0, 0.0},
        PoseGraphInformationMatrix2D{
          400.0, 0.0, 0.0, 1.0, 0.0, 400.0},
        PoseGraphConstraintType::kLoopClosure});

  PoseGraphOptimizationOptions options;
  options.loop_closure_huber_scale = 100.0;
  const auto result = graph.optimize(options);

  ASSERT_TRUE(result.success);
  EXPECT_NEAR(graph.nodes()[1].pose.x, 1.0, 1.0e-6);
  EXPECT_GT(graph.nodes()[1].pose.y, 0.99);
  EXPECT_LT(graph.nodes()[1].pose.y, 1.0);
}

TEST(PoseGraph2D, RejectsNonPositiveDefiniteInformation)
{
  PoseGraph2D graph;
  graph.addNode(Pose2D{});
  graph.addNode(Pose2D{1.0, 0.0, 0.0});

  EXPECT_THROW(
    graph.addConstraint(
      PoseGraphConstraint{
        0U,
        1U,
        Pose2D{1.0, 0.0, 0.0},
        PoseGraphInformationMatrix2D{
          1.0, 2.0, 0.0, 1.0, 0.0, 1.0},
        PoseGraphConstraintType::kSequential}),
    std::invalid_argument);
}

TEST(PoseGraph2D, RejectsDisconnectedGraph)
{
  PoseGraph2D graph;
  graph.addNode(Pose2D{});
  graph.addNode(Pose2D{1.0, 0.0, 0.0});

  EXPECT_THROW(graph.optimize(), std::invalid_argument);
}

TEST(PoseGraph2D, RollsBackNodesAndConstraints)
{
  PoseGraph2D graph;
  graph.addNode(Pose2D{});
  graph.addNode(Pose2D{1.0, 0.0, 0.0});
  const std::size_t constraint_id = graph.addConstraint(
    sequentialConstraint(0U, 1U, Pose2D{1.0, 0.0, 0.0}));

  EXPECT_THROW(graph.removeLastNode(), std::logic_error);

  graph.removeConstraint(constraint_id);
  graph.removeLastNode();

  EXPECT_EQ(graph.nodes().size(), 1U);
  EXPECT_TRUE(graph.constraints().empty());
  EXPECT_THROW(graph.removeConstraint(0U), std::out_of_range);
}

TEST(PoseGraph2D, ReplacesNodePosesAtomically)
{
  PoseGraph2D graph;
  graph.addNode(Pose2D{});
  graph.addNode(Pose2D{1.0, 0.0, 0.0});

  graph.setNodePoses(
    std::vector<Pose2D>{
        Pose2D{0.0, 0.0, 0.0},
        Pose2D{2.0, 1.0, 7.0}});
  EXPECT_NEAR(graph.nodes()[1].pose.x, 2.0, 1.0e-12);
  EXPECT_NEAR(graph.nodes()[1].pose.yaw, normalizeAngle(7.0), 1.0e-12);

  const auto poses_before = graph.nodes();
  EXPECT_THROW(
    graph.setNodePoses(
      std::vector<Pose2D>{
        Pose2D{},
        Pose2D{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}}),
    std::invalid_argument);
  EXPECT_EQ(graph.nodes()[1].pose.x, poses_before[1].pose.x);
  EXPECT_THROW(graph.setNodePoses(std::vector<Pose2D>{Pose2D{}}), std::invalid_argument);
}

}  // namespace
}  // namespace slam_robot_slam
