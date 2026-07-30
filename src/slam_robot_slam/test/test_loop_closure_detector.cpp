#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "slam_robot_slam/loop_closure_detector.hpp"

namespace slam_robot_slam
{
namespace
{

TEST(LoopClosureDetector, ExcludesRecentKeyframesAndSortsByDistance)
{
  const std::vector<Pose2D> poses{
    Pose2D{0.00, 0.00, 0.0},
    Pose2D{0.10, 0.00, 0.0},
    Pose2D{2.00, 0.00, 0.0},
    Pose2D{0.05, 0.00, 0.0},
    Pose2D{0.02, 0.00, 0.0}};
  LoopClosureCandidateParameters parameters;
  parameters.minimum_keyframe_separation = 2U;
  parameters.minimum_travel_distance = 2.0;
  parameters.search_radius = 0.2;
  parameters.maximum_candidates = 3U;

  const auto candidates =
    findLoopClosureCandidates(
    poses,
    std::vector<double>{0.0, 1.0, 2.0, 3.0, 4.0},
    4U,
    parameters);

  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_EQ(candidates[0].keyframe_id, 0U);
  EXPECT_NEAR(candidates[0].distance, 0.02, 1.0e-12);
  EXPECT_EQ(candidates[1].keyframe_id, 1U);
  EXPECT_NEAR(candidates[1].distance, 0.08, 1.0e-12);
}

TEST(LoopClosureDetector, LimitsCandidateCount)
{
  const std::vector<Pose2D> poses{
    Pose2D{0.00, 0.00, 0.0},
    Pose2D{0.10, 0.00, 0.0},
    Pose2D{0.20, 0.00, 0.0},
    Pose2D{0.15, 0.00, 0.0}};
  LoopClosureCandidateParameters parameters;
  parameters.minimum_keyframe_separation = 1U;
  parameters.minimum_travel_distance = 1.0;
  parameters.search_radius = 1.0;
  parameters.maximum_candidates = 2U;

  const auto candidates =
    findLoopClosureCandidates(
    poses,
    std::vector<double>{0.0, 1.0, 2.0, 3.0},
    3U,
    parameters);

  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_EQ(candidates[0].keyframe_id, 1U);
  EXPECT_EQ(candidates[1].keyframe_id, 2U);
}

TEST(LoopClosureDetector, WaitsForMinimumHistory)
{
  const std::vector<Pose2D> poses{
    Pose2D{0.0, 0.0, 0.0},
    Pose2D{0.0, 0.0, 0.0}};
  LoopClosureCandidateParameters parameters;
  parameters.minimum_keyframe_separation = 3U;

  EXPECT_TRUE(
    findLoopClosureCandidates(
      poses,
      std::vector<double>{0.0, 0.0},
      1U,
      parameters).empty());
}

TEST(LoopClosureDetector, RejectsInvalidInput)
{
  const std::vector<Pose2D> poses{Pose2D{}};
  LoopClosureCandidateParameters parameters;
  parameters.search_radius = 0.0;

  EXPECT_THROW(
    findLoopClosureCandidates(
      poses, std::vector<double>{0.0}, 0U, parameters),
    std::invalid_argument);
  EXPECT_THROW(
    findLoopClosureCandidates(
      poses,
      std::vector<double>{0.0},
      1U,
      LoopClosureCandidateParameters{}),
    std::out_of_range);
}

TEST(LoopClosureDetector, RejectsPureRotationWithoutTravel)
{
  const std::vector<Pose2D> poses{
    Pose2D{0.0, 0.0, 0.0},
    Pose2D{0.0, 0.0, 1.0},
    Pose2D{0.0, 0.0, 2.0},
    Pose2D{0.0, 0.0, 3.0}};
  LoopClosureCandidateParameters parameters;
  parameters.minimum_keyframe_separation = 2U;
  parameters.minimum_travel_distance = 2.0;

  EXPECT_TRUE(
    findLoopClosureCandidates(
      poses,
      std::vector<double>{0.0, 0.0, 0.0, 0.0},
      3U,
      parameters).empty());
}

}  // namespace
}  // namespace slam_robot_slam
