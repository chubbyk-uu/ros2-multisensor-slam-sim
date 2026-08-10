#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "slam_robot_slam_3d/height_aware_occupancy_grid.hpp"

namespace slam_robot_slam_3d
{
TEST(HeightAwareOccupancyGrid, ProjectsObstacleHitsAndGroundFreeSpace)
{
  auto scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  scan->push_back(pcl::PointXYZI{1.0F, 0.0F, 0.20F, 1.0F});
  scan->push_back(pcl::PointXYZI{2.0F, 0.0F, 0.00F, 1.0F});
  scan->push_back(pcl::PointXYZI{3.0F, 0.0F, 0.60F, 1.0F});
  GlobalKeyframe keyframe;
  keyframe.id = 0U;
  keyframe.filtered_scan = scan;
  HeightAwareOccupancyGrid grid({{}, 0.05, 0.45, 1U});
  grid.begin({keyframe}, {Eigen::Isometry3d::Identity()});
  EXPECT_TRUE(grid.processBatch());
  const auto snapshot = grid.snapshot();
  ASSERT_FALSE(snapshot.data.empty());
  const auto cell = [&](int x) {
      return snapshot.data[static_cast<std::size_t>(-snapshot.origin_cell_y) * snapshot.width +
               static_cast<std::size_t>(x - snapshot.origin_cell_x)];
    };
  // In-band returns occupy their endpoint.
  EXPECT_GE(cell(20), 50);
  // Ground returns contribute ray-traced free space but never a hit.
  EXPECT_LT(cell(40), 50);
  // High returns are not projected into a 2D obstacle map at all.
  EXPECT_LT(snapshot.origin_cell_x + static_cast<int>(snapshot.width), 60);
}

TEST(HeightAwareOccupancyGrid, AppendsNewKeyframesWithoutReplayingHistory)
{
  auto first_scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  first_scan->push_back(pcl::PointXYZI{1.0F, 0.0F, 0.20F, 1.0F});
  GlobalKeyframe first;
  first.id = 0U;
  first.filtered_scan = first_scan;
  HeightAwareOccupancyGrid grid({{}, 0.05, 0.45, 1U});
  grid.begin({first}, {Eigen::Isometry3d::Identity()});
  ASSERT_TRUE(grid.processBatch());

  auto second_scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  second_scan->push_back(pcl::PointXYZI{2.0F, 0.0F, 0.20F, 1.0F});
  GlobalKeyframe second;
  second.id = 1U;
  second.filtered_scan = second_scan;
  grid.append(second, Eigen::Isometry3d::Identity());

  const auto snapshot = grid.snapshot();
  const auto cell = [&](int x) {
      return snapshot.data[static_cast<std::size_t>(-snapshot.origin_cell_y) * snapshot.width +
               static_cast<std::size_t>(x - snapshot.origin_cell_x)];
    };
  EXPECT_GE(cell(20), 50);
  EXPECT_GE(cell(40), 50);
}

TEST(HeightAwareOccupancyGrid, PublishesOnlyTrinaryNavigationSemantics)
{
  auto ground_scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  ground_scan->push_back(pcl::PointXYZI{1.0F, 0.0F, 0.00F, 1.0F});
  GlobalKeyframe ground;
  ground.id = 0U;
  ground.filtered_scan = ground_scan;
  HeightAwareOccupancyGrid grid({{}, 0.05, 0.45, 1U});
  grid.begin({ground}, {Eigen::Isometry3d::Identity()});
  ASSERT_TRUE(grid.processBatch());

  const auto cell = [](const auto & snapshot, int x) {
      return snapshot.data[static_cast<std::size_t>(-snapshot.origin_cell_y) * snapshot.width +
               static_cast<std::size_t>(x - snapshot.origin_cell_x)];
    };
  EXPECT_EQ(cell(grid.snapshot(), 20), 40);
  EXPECT_EQ(cell(grid.navigationSnapshot(), 20), -1);

  // Four clear observations are required before a cell becomes navigation
  // free. Repeating the immutable keyframe here models later views along the
  // same ray without changing the integration contract.
  grid.append(ground, Eigen::Isometry3d::Identity());
  grid.append(ground, Eigen::Isometry3d::Identity());
  grid.append(ground, Eigen::Isometry3d::Identity());
  EXPECT_EQ(cell(grid.navigationSnapshot(), 20), 0);

  auto obstacle_scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  obstacle_scan->push_back(pcl::PointXYZI{2.0F, 0.0F, 0.20F, 1.0F});
  GlobalKeyframe obstacle;
  obstacle.id = 1U;
  obstacle.filtered_scan = obstacle_scan;
  HeightAwareOccupancyGrid obstacle_grid({{}, 0.05, 0.45, 1U});
  obstacle_grid.begin({obstacle}, {Eigen::Isometry3d::Identity()});
  ASSERT_TRUE(obstacle_grid.processBatch());
  EXPECT_EQ(cell(obstacle_grid.navigationSnapshot(), 40), 100);

  // A later contradictory ground ray weakens the obstacle posterior to 61,
  // but must never turn it into navigation free space.
  auto long_ground_scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  long_ground_scan->push_back(pcl::PointXYZI{2.0F, 0.0F, 0.00F, 1.0F});
  GlobalKeyframe long_ground;
  long_ground.id = 2U;
  long_ground.filtered_scan = long_ground_scan;
  obstacle_grid.append(long_ground, Eigen::Isometry3d::Identity());
  EXPECT_EQ(cell(obstacle_grid.snapshot(), 40), 61);
  EXPECT_EQ(cell(obstacle_grid.navigationSnapshot(), 40), -1);
}

TEST(HeightAwareOccupancyGrid, UsesConfiguredFreeEvidenceThreshold)
{
  auto scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  scan->push_back(pcl::PointXYZI{1.0F, 0.0F, 0.00F, 1.0F});
  GlobalKeyframe keyframe;
  keyframe.id = 0U;
  keyframe.filtered_scan = scan;
  HeightAwareOccupancyGrid grid({{}, 0.05, 0.45, 1U, 25, 65});
  grid.begin({keyframe}, {Eigen::Isometry3d::Identity()});
  ASSERT_TRUE(grid.processBatch());
  grid.append(keyframe, Eigen::Isometry3d::Identity());
  grid.append(keyframe, Eigen::Isometry3d::Identity());

  const auto cell = [](const auto & snapshot) {
      return snapshot.data[static_cast<std::size_t>(-snapshot.origin_cell_y) * snapshot.width +
               static_cast<std::size_t>(20 - snapshot.origin_cell_x)];
    };
  EXPECT_EQ(cell(grid.snapshot()), 23);
  EXPECT_EQ(cell(grid.navigationSnapshot()), 0);
}
}  // namespace slam_robot_slam_3d
