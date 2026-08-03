#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "slam_robot_slam/pose_covariance_2d.hpp"

namespace slam_robot_slam
{
namespace
{

TEST(PoseCovariance2D, RepresentsWeakDirectionAnisotropically)
{
  const PoseCovariance2D covariance = matchedPoseCovariance(
    1U, Point2D{1.0F, 1.0F}, PoseCovariance2DParameters{});

  EXPECT_GT(covariance.xy, 0.0);
  EXPECT_NEAR(covariance.xx, covariance.yy, 1.0e-12);
  EXPECT_NEAR(covariance.xx + covariance.xy, 0.09, 1.0e-8);
  EXPECT_NEAR(covariance.xx - covariance.xy, 0.0025, 1.0e-8);
}

TEST(PoseCovariance2D, AccumulatesIncrementalDeadReckoningNoise)
{
  const PoseCovariance2DParameters parameters;
  PoseCovariance2D covariance = matchedPoseCovariance(
    2U, Point2D{1.0F, 0.0F}, parameters);
  covariance = propagateDeadReckoningCovariance(
    covariance, 0.25, 0.10, parameters);
  covariance = propagateDeadReckoningCovariance(
    covariance, 0.25, 0.10, parameters);

  EXPECT_NEAR(covariance.xx, 0.0075, 1.0e-12);
  EXPECT_NEAR(covariance.yy, 0.0075, 1.0e-12);
  EXPECT_NEAR(covariance.yaw_yaw, 0.0045, 1.0e-12);
}

TEST(PoseCovariance2D, RejectsInvalidInputs)
{
  PoseCovariance2DParameters parameters;
  parameters.dead_reckoning_translation_stddev_per_sqrt_meter = 0.0;
  EXPECT_THROW(
    validatePoseCovariance2DParameters(parameters),
    std::invalid_argument);
  EXPECT_THROW(
    matchedPoseCovariance(
      1U, Point2D{0.0F, 0.0F}, PoseCovariance2DParameters{}),
    std::invalid_argument);
  EXPECT_THROW(
    propagateDeadReckoningCovariance(
      PoseCovariance2D{},
      std::numeric_limits<double>::quiet_NaN(),
      0.0,
      PoseCovariance2DParameters{}),
    std::invalid_argument);
}

}  // namespace
}  // namespace slam_robot_slam
