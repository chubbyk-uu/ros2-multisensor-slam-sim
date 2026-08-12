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

TEST(GoalResponse, ALateAcceptanceIsCancelledRatherThanIgnored)
{
  // The race no Gazebo run will show: the explorer gave up waiting, the server
  // accepted anyway, and the response arrives afterwards. Returning early on
  // staleness drops the only handle that could cancel it, so the robot drives
  // to a frontier nobody is tracking while the explorer sends the next goal.
  EXPECT_EQ(
    classifyGoalResponse(true, false), GoalResponse::kStaleAccepted);
}

TEST(GoalResponse, ALateRejectionNeedsNothing)
{
  // Nothing is running, so there is nothing to cancel; cancelling a rejected
  // goal would be an error against the server.
  EXPECT_EQ(
    classifyGoalResponse(false, false), GoalResponse::kStaleRejected);
}

TEST(GoalResponse, ATimelyResponseIsHandledNormally)
{
  EXPECT_EQ(classifyGoalResponse(true, true), GoalResponse::kAccepted);
  EXPECT_EQ(classifyGoalResponse(false, true), GoalResponse::kRejected);
}

TEST(NavigationAvailability, AnUnansweredGoalRunsTheSameBudgetAsARejectedOne)
{
  NavigationAvailability availability;
  const auto start = Clock::now();
  availability.observe(true, start, kBudget);

  // A server that never replies has not taken the goal either. Counting this
  // as usable would restart the budget on every retry and the fault would
  // never fire.
  availability.observeGoalUnanswered();

  ASSERT_EQ(
    availability.observe(true, start + std::chrono::seconds(1), kBudget),
    NavigationAvailability::Status::kWaiting);
  EXPECT_EQ(
    availability.observe(true, start + std::chrono::seconds(7), kBudget),
    NavigationAvailability::Status::kLost);
  EXPECT_STREQ(availability.failureCode(), kFailureCodeNav2StartupTimeout);
}

TEST(NavigationAvailability, RetriesDoNotRestartTheBudget)
{
  NavigationAvailability availability;
  const auto start = Clock::now();
  availability.observe(true, start, kBudget);

  // Each retry observes the failure again. If that rearmed the deadline the
  // explorer would retry for ever, which is the failure mode the budget was
  // added to prevent.
  for (int second = 0; second < 7; ++second) {
    availability.observeGoalUnanswered();
    availability.observe(true, start + std::chrono::seconds(second), kBudget);
  }

  EXPECT_EQ(availability.status(), NavigationAvailability::Status::kLost);
  EXPECT_EQ(availability.unansweredGoals(), 7U);
  EXPECT_EQ(availability.rejectedGoals(), 0U);
}

TEST(NavigationAvailability, RejectionsAndSilenceAreCountedApart)
{
  NavigationAvailability availability;
  availability.observeGoalRejected();
  availability.observeGoalUnanswered();
  availability.observeGoalUnanswered();

  // Same budget, different diagnosis: "refused" and "never replied" send an
  // investigation to different places.
  EXPECT_EQ(availability.rejectedGoals(), 1U);
  EXPECT_EQ(availability.unansweredGoals(), 2U);
}

}  // namespace
}  // namespace slam_robot_navigation
