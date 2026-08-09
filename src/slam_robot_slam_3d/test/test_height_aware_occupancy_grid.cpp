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
}  // namespace slam_robot_slam_3d
