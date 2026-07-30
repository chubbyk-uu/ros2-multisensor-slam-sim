#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "slam_robot_slam/container_utils.hpp"

namespace slam_robot_slam
{
namespace
{

TEST(ContainerUtils, LeavesSequenceBelowLimitUntouched)
{
  std::vector<int> values{1, 2, 3};

  trimOldestInBatches(values, 3U);

  EXPECT_EQ(values, (std::vector<int>{1, 2, 3}));
}

TEST(ContainerUtils, RemovesOldestItemsInAmortizedBatch)
{
  std::vector<int> values;
  for (int value = 0; value <= 100; ++value) {
    values.push_back(value);
  }

  trimOldestInBatches(values, 100U);

  ASSERT_EQ(values.size(), 90U);
  EXPECT_EQ(values.front(), 11);
  EXPECT_EQ(values.back(), 100);
}

TEST(ContainerUtils, SupportsSmallLimitsAndRejectsZero)
{
  std::vector<int> values{1, 2};

  trimOldestInBatches(values, 1U);
  EXPECT_EQ(values, (std::vector<int>{2}));
  EXPECT_THROW(trimOldestInBatches(values, 0U), std::invalid_argument);
}

}  // namespace
}  // namespace slam_robot_slam
