#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "slam_robot_slam/pose2d.hpp"

namespace slam_robot_slam
{
namespace
{

TEST(Pose2D, CompositionAndRelativePoseAreConsistent)
{
  const Pose2D first{1.0, -0.5, 0.4};
  const Pose2D increment{0.3, 0.1, -0.2};
  const auto second = composePoses(first, increment);
  const auto recovered = relativePose(first, second);

  EXPECT_NEAR(recovered.x, increment.x, 1.0e-6);
  EXPECT_NEAR(recovered.y, increment.y, 1.0e-6);
  EXPECT_NEAR(recovered.yaw, increment.yaw, 1.0e-6);
}

TEST(Pose2D, MapToOdomCorrectionRecoversMatchedBasePose)
{
  const Pose2D odom_from_base{2.4, -0.7, 0.35};
  const Pose2D map_from_base{2.1, -0.5, 0.28};
  const auto map_from_odom = composePoses(
    map_from_base,
    inversePose(odom_from_base));
  const auto recovered_map_from_base = composePoses(
    map_from_odom,
    odom_from_base);

  EXPECT_NEAR(recovered_map_from_base.x, map_from_base.x, 1.0e-9);
  EXPECT_NEAR(recovered_map_from_base.y, map_from_base.y, 1.0e-9);
  EXPECT_NEAR(recovered_map_from_base.yaw, map_from_base.yaw, 1.0e-9);
}

TEST(Pose2D, NormalizesLargeAnglesInBoundedTime)
{
  constexpr double kPi = 3.14159265358979323846;
  const double normalized = normalizeAngle(1.0e20);

  EXPECT_TRUE(std::isfinite(normalized));
  EXPECT_GE(normalized, -kPi);
  EXPECT_LE(normalized, kPi);
  EXPECT_TRUE(
    std::isnan(normalizeAngle(std::numeric_limits<double>::infinity())));
}

TEST(Pose2D, DetectsNonFinitePose)
{
  EXPECT_TRUE(isFinitePose(Pose2D{1.0, 2.0, 0.3}));
  EXPECT_FALSE(
    isFinitePose(
      Pose2D{
        std::numeric_limits<double>::quiet_NaN(),
        2.0,
        0.3}));
}

TEST(Pose2D, InterpolatesAcrossAngleWrap)
{
  const Pose2D interpolated = interpolatePoses(
    Pose2D{0.0, 1.0, 3.10},
    Pose2D{2.0, 3.0, -3.10},
    0.5);

  EXPECT_NEAR(interpolated.x, 1.0, 1.0e-12);
  EXPECT_NEAR(interpolated.y, 2.0, 1.0e-12);
  EXPECT_NEAR(std::abs(interpolated.yaw), 3.14159265358979323846, 1.0e-3);
  EXPECT_THROW(
    interpolatePoses(Pose2D{}, Pose2D{}, 1.1),
    std::invalid_argument);
}

}  // namespace
}  // namespace slam_robot_slam
