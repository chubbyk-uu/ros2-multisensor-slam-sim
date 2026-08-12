// Copyright 2026 Jerry

#include <gtest/gtest.h>

#include "slam_robot_navigation/navigation_availability.hpp"

namespace slam_robot_navigation
{

TEST(NavigationAvailability, WaitsUntilBothServersAreInitiallyReady)
{
  NavigationAvailability availability;

  EXPECT_EQ(
    availability.observe(false, false), NavigationAvailability::Status::kWaiting);
  EXPECT_EQ(
    availability.observe(true, false), NavigationAvailability::Status::kWaiting);
  EXPECT_EQ(
    availability.observe(true, true), NavigationAvailability::Status::kReady);
}

TEST(NavigationAvailability, BecomesPermanentlyLostAfterRuntimeDropout)
{
  NavigationAvailability availability;
  ASSERT_EQ(
    availability.observe(true, true), NavigationAvailability::Status::kReady);

  EXPECT_EQ(
    availability.observe(false, true), NavigationAvailability::Status::kLost);
  EXPECT_EQ(
    availability.observe(true, true), NavigationAvailability::Status::kLost);
}

TEST(NavigationAvailability, RejectedGoalIsAnInfrastructureFault)
{
  NavigationAvailability availability;
  ASSERT_EQ(
    availability.observe(true, true), NavigationAvailability::Status::kReady);

  EXPECT_EQ(
    availability.goalRejected(), NavigationAvailability::Status::kLost);
}

}  // namespace slam_robot_navigation
