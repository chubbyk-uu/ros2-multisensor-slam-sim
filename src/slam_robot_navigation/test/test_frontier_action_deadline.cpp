#include <chrono>

#include <gtest/gtest.h>

#include "slam_robot_navigation/frontier_action_deadline.hpp"

namespace slam_robot_navigation
{

TEST(FrontierActionDeadline, ExpiresOnlyAfterItsArmedTimeout)
{
  FrontierActionDeadline deadline;
  const auto start = FrontierActionDeadline::Clock::time_point{};
  EXPECT_FALSE(deadline.armed());
  EXPECT_FALSE(deadline.expired(start));

  deadline.arm(start, std::chrono::seconds(15));
  EXPECT_TRUE(deadline.armed());
  EXPECT_FALSE(deadline.expired(start + std::chrono::seconds(15)));
  EXPECT_TRUE(deadline.expired(start + std::chrono::seconds(16)));

  deadline.disarm();
  EXPECT_FALSE(deadline.armed());
  EXPECT_FALSE(deadline.expired(start + std::chrono::hours(1)));
}

}  // namespace slam_robot_navigation
