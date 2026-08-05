#include <stdexcept>

#include <gtest/gtest.h>

#include "slam_robot_slam_3d/match_failure_recovery.hpp"

namespace slam_robot_slam_3d
{

TEST(MatchFailureRecovery, TriggersOnlyAtConsecutiveRegistrationFailureLimit)
{
  MatchFailureRecovery recovery(3U);

  EXPECT_FALSE(recovery.observe(ScanToMapStatus::kNotConverged));
  EXPECT_FALSE(recovery.observe(ScanToMapStatus::kFitnessTooHigh));
  EXPECT_TRUE(recovery.observe(ScanToMapStatus::kCorrectionTooLarge));
  EXPECT_EQ(recovery.consecutiveFailures(), 0U);
  EXPECT_EQ(recovery.reinitializationCount(), 1U);
}

TEST(MatchFailureRecovery, SuccessResetsSequenceAndInvalidInputDoesNotPolluteIt)
{
  MatchFailureRecovery recovery(2U);

  EXPECT_FALSE(recovery.observe(ScanToMapStatus::kNotConverged));
  EXPECT_FALSE(recovery.observe(ScanToMapStatus::kSuccess));
  EXPECT_FALSE(recovery.observe(ScanToMapStatus::kInvalidInput));
  EXPECT_FALSE(recovery.observe(ScanToMapStatus::kNotConverged));
  EXPECT_TRUE(recovery.observe(ScanToMapStatus::kInsufficientCorrespondences));
  EXPECT_EQ(recovery.reinitializationCount(), 1U);
}

TEST(MatchFailureRecovery, RejectsZeroLimit)
{
  EXPECT_THROW((void)MatchFailureRecovery{0U}, std::invalid_argument);
}

}  // namespace slam_robot_slam_3d
