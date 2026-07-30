#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "slam_robot_slam/correlative_scan_matcher.hpp"

namespace
{

std::vector<slam_robot_slam::Point2D> makeRoomCorner()
{
  std::vector<slam_robot_slam::Point2D> points;
  for (int index = 0; index <= 80; ++index) {
    points.push_back(
      slam_robot_slam::Point2D{
        3.0F,
        static_cast<float>(-2.0 + 0.05 * index)});
  }
  for (int index = 1; index <= 100; ++index) {
    points.push_back(
      slam_robot_slam::Point2D{
        static_cast<float>(3.0 - 0.05 * index),
        2.0F});
  }
  for (int index = 1; index <= 50; ++index) {
    points.push_back(
      slam_robot_slam::Point2D{
        -2.0F,
        static_cast<float>(2.0 - 0.05 * index)});
  }
  return points;
}

TEST(CorrelativeScanMatcher, RecoversPoseAgainstLocalMap)
{
  const auto reference = makeRoomCorner();
  const slam_robot_slam::Pose2D expected{0.10, -0.06, 0.04};
  const auto current_from_reference =
    slam_robot_slam::inversePose(expected);
  std::vector<slam_robot_slam::Point2D> current;
  current.reserve(reference.size());
  for (const auto & point : reference) {
    current.push_back(
      slam_robot_slam::transformPoint(current_from_reference, point));
  }

  slam_robot_slam::CorrelativeScanMatcherParameters parameters;
  parameters.grid_resolution = 0.02;
  parameters.smear_deviation = 0.04;
  parameters.coarse_linear_resolution = 0.02;
  parameters.coarse_angular_resolution = 0.02;
  parameters.fine_linear_resolution = 0.005;
  parameters.fine_angular_resolution = 0.005;
  parameters.minimum_score = 0.50;
  parameters.minimum_matched_points = 80U;
  const auto result = slam_robot_slam::matchCorrelative(
    reference,
    current,
    slam_robot_slam::Pose2D{0.05, -0.02, 0.02},
    parameters);

  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.pose.x, expected.x, 0.011);
  EXPECT_NEAR(result.pose.y, expected.y, 0.011);
  EXPECT_NEAR(result.pose.yaw, expected.yaw, 0.011);
  EXPECT_GT(result.score, 0.80);
  EXPECT_GT(result.support_fraction, 0.90);
  EXPECT_GT(result.evaluated_candidates, 1000U);
}

TEST(CorrelativeScanMatcher, RejectsScanWithoutOverlap)
{
  const auto reference = makeRoomCorner();
  std::vector<slam_robot_slam::Point2D> current{
    {20.0F, 20.0F},
    {20.1F, 20.0F},
    {20.2F, 20.0F}};

  slam_robot_slam::CorrelativeScanMatcherParameters parameters;
  parameters.minimum_matched_points = 3U;
  const auto result = slam_robot_slam::matchCorrelative(
    reference,
    current,
    slam_robot_slam::Pose2D{},
    parameters);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.matched_points, 0U);
}

TEST(CorrelativeScanMatcher, ScoresSupportedOverlapSeparately)
{
  const auto reference = makeRoomCorner();
  std::vector<slam_robot_slam::Point2D> current = reference;
  for (int index = 0; index < 200; ++index) {
    current.push_back(
      slam_robot_slam::Point2D{
        20.0F + static_cast<float>(index) * 0.01F,
        20.0F});
  }

  slam_robot_slam::CorrelativeScanMatcherParameters parameters;
  parameters.minimum_score = 0.8;
  parameters.minimum_support_fraction = 0.4;
  parameters.minimum_matched_points = 100U;
  const auto result = slam_robot_slam::matchCorrelative(
    reference,
    current,
    slam_robot_slam::Pose2D{},
    parameters);

  EXPECT_TRUE(result.success);
  EXPECT_GT(result.score, 0.9);
  EXPECT_GT(result.support_fraction, 0.4);
  EXPECT_LT(result.support_fraction, 0.6);
}

TEST(CorrelativeScanMatcher, RejectsInvalidParameters)
{
  slam_robot_slam::CorrelativeScanMatcherParameters parameters;
  parameters.grid_resolution = 0.0;

  EXPECT_THROW(
    slam_robot_slam::matchCorrelative(
      makeRoomCorner(),
      makeRoomCorner(),
      slam_robot_slam::Pose2D{},
      parameters),
    std::invalid_argument);

  parameters = slam_robot_slam::CorrelativeScanMatcherParameters{};
  parameters.minimum_score =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    slam_robot_slam::validateCorrelativeScanMatcherParameters(parameters),
    std::invalid_argument);

  parameters = slam_robot_slam::CorrelativeScanMatcherParameters{};
  parameters.minimum_support_fraction = 0.0;
  EXPECT_THROW(
    slam_robot_slam::validateCorrelativeScanMatcherParameters(parameters),
    std::invalid_argument);
}

TEST(CorrelativeScanMatcher, RejectsNonFinitePoints)
{
  auto current = makeRoomCorner();
  current.front().x = std::numeric_limits<float>::quiet_NaN();

  EXPECT_THROW(
    slam_robot_slam::matchCorrelative(
      makeRoomCorner(),
      current,
      slam_robot_slam::Pose2D{},
      slam_robot_slam::CorrelativeScanMatcherParameters{}),
    std::invalid_argument);
}

}  // namespace
