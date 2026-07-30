#include <gtest/gtest.h>

#include <vector>

#include "slam_robot_slam/latest_snapshot_queue.hpp"

namespace slam_robot_slam
{
namespace
{

TEST(LatestSnapshotQueue, KeepsActiveSnapshotStableWhileWorkRuns)
{
  LatestSnapshotQueue<std::vector<int>> queue;
  queue.request({1, 2});
  queue.request({1, 2, 3});

  EXPECT_EQ(queue.active(), (std::vector<int>{1, 2}));
  EXPECT_EQ(queue.queued(), (std::vector<int>{1, 2, 3}));
}

TEST(LatestSnapshotQueue, CoalescesPendingRequestsToLatestSnapshot)
{
  LatestSnapshotQueue<std::vector<int>> queue;
  queue.request({1});
  queue.request({1, 2});
  queue.request({1, 2, 3});

  ASSERT_TRUE(queue.completeActive());
  EXPECT_EQ(queue.active(), (std::vector<int>{1, 2, 3}));
  EXPECT_FALSE(queue.hasQueued());
  EXPECT_FALSE(queue.completeActive());
  EXPECT_FALSE(queue.hasActive());
}

TEST(LatestSnapshotQueue, ClearsActiveAndQueuedWork)
{
  LatestSnapshotQueue<std::vector<int>> queue;
  queue.request({1});
  queue.request({2});

  queue.clear();

  EXPECT_FALSE(queue.hasActive());
  EXPECT_FALSE(queue.hasQueued());
}

}  // namespace
}  // namespace slam_robot_slam
