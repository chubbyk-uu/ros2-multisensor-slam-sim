// Copyright 2026 Jerry

#include <gtest/gtest.h>

#include <chrono>

#include "slam_robot_navigation/empty_frontier_counter.hpp"

namespace slam_robot_navigation
{
namespace
{

using Clock = EmptyFrontierCounter::Clock;
constexpr auto kStale = std::chrono::seconds(10);
const Clock::time_point kStart{};

}  // namespace

TEST(EmptyFrontierCounter, PollingOneUnchangedMapFasterThanItRepublishesCountsOnce)
{
  EmptyFrontierCounter counter(kStale);
  for (int second = 0; second < 9; ++second) {
    counter.observeEmpty(31U, kStart + std::chrono::seconds(second));
  }

  EXPECT_EQ(counter.count(), 1U);
}

TEST(EmptyFrontierCounter, EachNewMapAgreeingCountsOnce)
{
  EmptyFrontierCounter counter(kStale);
  counter.observeEmpty(31U, kStart);
  counter.observeEmpty(31U, kStart + std::chrono::seconds(1));
  counter.observeEmpty(32U, kStart + std::chrono::seconds(2));
  counter.observeEmpty(32U, kStart + std::chrono::seconds(3));
  counter.observeEmpty(33U, kStart + std::chrono::seconds(4));

  EXPECT_EQ(counter.count(), 3U);
}

TEST(EmptyFrontierCounter, AMapThatStoppedRepublishingStillReachesTheThreshold)
{
  // A finished exploration stops the robot, which stops the map. Requiring a
  // new revision every time would make completion unreachable exactly then.
  EmptyFrontierCounter counter(kStale);
  for (int second = 0; second <= 60; ++second) {
    counter.observeEmpty(72U, kStart + std::chrono::seconds(second));
  }

  EXPECT_GE(counter.count(), 5U);
}

TEST(EmptyFrontierCounter, StalenessIsMeasuredFromTheLastCountedLook)
{
  EmptyFrontierCounter counter(kStale);
  counter.observeEmpty(72U, kStart);
  counter.observeEmpty(72U, kStart + std::chrono::seconds(9));
  EXPECT_EQ(counter.count(), 1U);

  counter.observeEmpty(72U, kStart + std::chrono::seconds(10));
  EXPECT_EQ(counter.count(), 2U);

  counter.observeEmpty(72U, kStart + std::chrono::seconds(19));
  EXPECT_EQ(counter.count(), 2U);
}

TEST(EmptyFrontierCounter, ASingleFrontierResetsTheEvidence)
{
  EmptyFrontierCounter counter(kStale);
  counter.observeEmpty(31U, kStart);
  counter.observeEmpty(32U, kStart + std::chrono::seconds(1));
  counter.observeFrontier();

  EXPECT_EQ(counter.count(), 0U);

  // The reset also forgets which revision was last seen, so the very next
  // empty observation counts even though it repeats that revision.
  counter.observeEmpty(32U, kStart + std::chrono::seconds(2));
  EXPECT_EQ(counter.count(), 1U);
}

}  // namespace slam_robot_navigation
