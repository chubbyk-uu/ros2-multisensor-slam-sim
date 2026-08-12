// Copyright 2026 Jerry

#include <gtest/gtest.h>

#include <chrono>

#include "slam_robot_navigation/navigation_availability.hpp"

namespace slam_robot_navigation
{
namespace
{

using Clock = NavigationAvailability::Clock;
constexpr auto kBudget = std::chrono::seconds(5);

TEST(NavigationAvailability, ARejectedGoalInsideTheBudgetIsARetryNotAFault)
{
  NavigationAvailability availability;
  const auto start = Clock::now();
  ASSERT_EQ(
    availability.observe(true, start, kBudget), NavigationAvailability::Status::kWaiting);

  // This is the Nav2 lifecycle window: the server is on the graph because it
  // was created in on_configure, and it rejects everything until on_activate.
  availability.observeGoalRejected();

  EXPECT_EQ(
    availability.observe(true, start + std::chrono::seconds(1), kBudget),
    NavigationAvailability::Status::kWaiting);
  EXPECT_EQ(
    availability.observe(true, start + std::chrono::seconds(4), kBudget),
    NavigationAvailability::Status::kWaiting);
  EXPECT_EQ(availability.rejectedGoals(), 1U);
}

TEST(NavigationAvailability, AnAcceptedNavigationGoalClearsTheBudget)
{
  NavigationAvailability availability;
  const auto start = Clock::now();
  availability.observe(true, start, kBudget);
  availability.observeGoalRejected();
  availability.observe(true, start + std::chrono::seconds(4), kBudget);

  availability.observeNavigationGoalAccepted();

  EXPECT_EQ(
    availability.observe(true, start + std::chrono::seconds(20), kBudget),
    NavigationAvailability::Status::kOperational);
  EXPECT_TRUE(availability.everOperational());
}

TEST(NavigationAvailability, SustainedRejectionBeyondTheBudgetIsAStartupTimeout)
{
  NavigationAvailability availability;
  const auto start = Clock::now();
  availability.observe(true, start, kBudget);
  availability.observeGoalRejected();

  // The budget starts when the trouble is first observed, not when it began:
  // an unobserved condition has not cost the run anything yet.
  ASSERT_EQ(
    availability.observe(true, start + std::chrono::seconds(1), kBudget),
    NavigationAvailability::Status::kWaiting);
  EXPECT_EQ(
    availability.observe(true, start + std::chrono::seconds(7), kBudget),
    NavigationAvailability::Status::kLost);
  // Never took a navigation goal, so however long the servers were visible,
  // this run never started.
  EXPECT_STREQ(availability.failureCode(), kFailureCodeNav2StartupTimeout);
}

TEST(NavigationAvailability, LosingAnOperationalChainIsARuntimeLoss)
{
  NavigationAvailability availability;
  const auto start = Clock::now();
  availability.observe(true, start, kBudget);
  availability.observeNavigationGoalAccepted();
  ASSERT_EQ(
    availability.observe(true, start, kBudget), NavigationAvailability::Status::kOperational);

  EXPECT_EQ(
    availability.observe(false, start + std::chrono::seconds(1), kBudget),
    NavigationAvailability::Status::kWaiting);
  EXPECT_EQ(
    availability.observe(false, start + std::chrono::seconds(7), kBudget),
    NavigationAvailability::Status::kLost);
  EXPECT_STREQ(availability.failureCode(), kFailureCodeNav2RuntimeLost);
}

TEST(NavigationAvailability, ABriefDisappearanceIsNotAFault)
{
  NavigationAvailability availability;
  const auto start = Clock::now();
  availability.observe(true, start, kBudget);
  availability.observeNavigationGoalAccepted();

  // One cycle of graph churn used to be terminal. The budget exists so that a
  // discovery hiccup costs a log line rather than a whole run.
  EXPECT_EQ(
    availability.observe(false, start + std::chrono::seconds(1), kBudget),
    NavigationAvailability::Status::kWaiting);
  EXPECT_EQ(
    availability.observe(true, start + std::chrono::seconds(2), kBudget),
    NavigationAvailability::Status::kOperational);
  EXPECT_EQ(
    availability.observe(true, start + std::chrono::seconds(60), kBudget),
    NavigationAvailability::Status::kOperational);
}

TEST(NavigationAvailability, APlannerGoalAloneDoesNotProveTheChainIsOperational)
{
  NavigationAvailability availability;
  const auto start = Clock::now();
  availability.observe(true, start, kBudget);

  // planner_server and bt_navigator activate independently, so a taken path
  // request says nothing about whether a navigation goal would be taken.
  availability.observePlannerGoalAccepted();
  availability.observeGoalRejected();

  EXPECT_FALSE(availability.everOperational());
  availability.observe(true, start + std::chrono::seconds(1), kBudget);
  EXPECT_EQ(
    availability.observe(true, start + std::chrono::seconds(7), kBudget),
    NavigationAvailability::Status::kLost);
  EXPECT_STREQ(availability.failureCode(), kFailureCodeNav2StartupTimeout);
}

TEST(NavigationAvailability, TheLostStateIsTerminal)
{
  NavigationAvailability availability;
  const auto start = Clock::now();
  availability.observe(false, start, kBudget);
  ASSERT_EQ(
    availability.observe(false, start + std::chrono::seconds(6), kBudget),
    NavigationAvailability::Status::kLost);

  availability.observeNavigationGoalAccepted();
  EXPECT_EQ(
    availability.observe(true, start + std::chrono::seconds(7), kBudget),
    NavigationAvailability::Status::kLost);
}

}  // namespace
}  // namespace slam_robot_navigation
