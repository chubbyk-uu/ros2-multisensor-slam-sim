#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "slam_robot_slam/point_to_line_icp.hpp"

namespace
{

std::vector<slam_robot_slam::Point2D> makeAsymmetricReference()
{
  std::vector<slam_robot_slam::Point2D> points;
  for (int index = 0; index <= 80; ++index) {
    points.push_back(
      slam_robot_slam::Point2D{
        2.0F,
        static_cast<float>(-2.0 + 0.05 * index)});
  }
  for (int index = 1; index <= 60; ++index) {
    points.push_back(
      slam_robot_slam::Point2D{
        static_cast<float>(2.0 - 0.05 * index),
        2.0F});
  }
  for (int index = 1; index <= 40; ++index) {
    points.push_back(
      slam_robot_slam::Point2D{
        -1.0F,
        static_cast<float>(2.0 - 0.05 * index)});
  }
  return points;
}

TEST(PointToLineIcp, RecoversKnownRelativePose)
{
  const auto reference = makeAsymmetricReference();
  const slam_robot_slam::Pose2D expected{0.08, -0.04, 0.03};
  const auto current_from_reference =
    slam_robot_slam::inversePose(expected);

  std::vector<slam_robot_slam::Point2D> current;
  current.reserve(reference.size());
  for (const auto & point : reference) {
    current.push_back(
      slam_robot_slam::transformPoint(current_from_reference, point));
  }

  slam_robot_slam::IcpParameters parameters;
  parameters.maximum_correspondence_distance = 0.30;
  parameters.maximum_neighbor_distance = 0.15;
  parameters.minimum_correspondences = 60U;
  const auto result = slam_robot_slam::matchPointToLineIcp(
    reference,
    current,
    slam_robot_slam::Pose2D{0.05, -0.02, 0.015},
    parameters);

  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.pose.x, expected.x, 2.0e-3);
  EXPECT_NEAR(result.pose.y, expected.y, 2.0e-3);
  EXPECT_NEAR(result.pose.yaw, expected.yaw, 2.0e-3);
  EXPECT_LT(result.mean_absolute_error, 1.0e-3);
}

TEST(PointToLineIcp, RejectsInsufficientGeometry)
{
  const std::vector<slam_robot_slam::Point2D> points{
    {0.0F, 0.0F},
    {1.0F, 0.0F},
    {2.0F, 0.0F}};
  slam_robot_slam::IcpParameters parameters;
  parameters.minimum_correspondences = 10U;

  const auto result = slam_robot_slam::matchPointToLineIcp(
    points, points, slam_robot_slam::Pose2D{}, parameters);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.iterations, 0U);
}

}  // namespace
