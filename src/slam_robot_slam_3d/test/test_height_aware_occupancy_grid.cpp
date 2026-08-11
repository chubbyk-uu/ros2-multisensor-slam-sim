#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "slam_robot_slam_3d/height_aware_occupancy_grid.hpp"

namespace slam_robot_slam_3d
{
namespace
{

// Built field by field on purpose. Positional aggregate initialisation reads
// compactly but silently reassigns every later value when a field is inserted
// in the middle, which is how a range limit once became a free threshold.
HeightAwareOccupancyGridParameters makeParameters(
  double maximum_ray_range = 100.0, std::int8_t free_maximum = 20,
  std::int8_t occupied_minimum = 65)
{
  HeightAwareOccupancyGridParameters parameters;
  parameters.projection.minimum_obstacle_height = 0.05;
  parameters.projection.maximum_obstacle_height = 0.45;
  parameters.keyframes_per_batch = 1U;
  parameters.projection.maximum_ray_range = maximum_ray_range;
  parameters.free_maximum = free_maximum;
  parameters.occupied_minimum = occupied_minimum;
  return parameters;
}

}  // namespace

TEST(HeightAwareOccupancyGrid, SelectsOnlyPersistentPlanarEvidence)
{
  pcl::PointCloud<pcl::PointXYZI> input;
  input.push_back(pcl::PointXYZI{1.0F, 0.0F, -0.20F, 1.0F});
  input.push_back(pcl::PointXYZI{2.0F, 0.0F, 0.20F, 1.0F});
  input.push_back(pcl::PointXYZI{3.0F, 0.0F, 0.60F, 1.0F});
  input.push_back(pcl::PointXYZI{9.0F, 0.0F, 0.20F, 1.0F});
  OccupancyProjectionContract contract;
  contract.maximum_ray_range = 8.0;

  const auto selected = selectPersistentOccupancyEvidence(
    input, Eigen::Isometry3d::Identity(), contract);

  ASSERT_EQ(selected.size(), 2U);
  EXPECT_FLOAT_EQ(selected[0].z, -0.20F);
  EXPECT_FLOAT_EQ(selected[1].z, 0.20F);
}

TEST(HeightAwareOccupancyGrid, PreservesAllEvidenceForNonPlanarMotion)
{
  pcl::PointCloud<pcl::PointXYZI> input;
  input.push_back(pcl::PointXYZI{1.0F, 0.0F, 0.60F, 1.0F});
  input.push_back(pcl::PointXYZI{9.0F, 0.0F, 0.20F, 1.0F});
  OccupancyProjectionContract contract;
  contract.force_planar_motion = false;

  const auto selected = selectPersistentOccupancyEvidence(
    input, Eigen::Isometry3d::Identity(), contract);

  EXPECT_EQ(selected.size(), input.size());
}

TEST(HeightAwareOccupancyGrid, MeasuresRangeInTheProjectionPlane)
{
  pcl::PointCloud<pcl::PointXYZI> input;
  input.push_back(pcl::PointXYZI{9.0F, 0.0F, 0.0F, 1.0F});
  OccupancyProjectionContract contract;
  Eigen::Isometry3d base_from_sensor = Eigen::Isometry3d::Identity();
  base_from_sensor.linear() =
    Eigen::AngleAxisd(std::acos(-1.0) / 3.0, Eigen::Vector3d::UnitY()).toRotationMatrix();

  const auto selected = selectPersistentOccupancyEvidence(
    input, base_from_sensor, contract);

  // Raw sensor x is 9 m, but its projected planar range after the sensor
  // extrinsic is 4.5 m. The shared classifier must therefore retain it.
  EXPECT_EQ(selected.size(), 1U);
}

TEST(HeightAwareOccupancyGrid, ProjectionContractComparisonCoversPersistentSemantics)
{
  OccupancyProjectionContract first;
  auto changed = first;
  EXPECT_TRUE(occupancyProjectionContractsMatch(first, changed));
  changed.maximum_ray_range = 10.0;
  EXPECT_FALSE(occupancyProjectionContractsMatch(first, changed));
  changed = first;
  changed.maximum_obstacle_height = 0.50;
  EXPECT_FALSE(occupancyProjectionContractsMatch(first, changed));
  changed = first;
  changed.input_voxel_leaf_size = 0.10;
  EXPECT_FALSE(occupancyProjectionContractsMatch(first, changed));
  changed = first;
  changed.force_planar_motion = false;
  EXPECT_FALSE(occupancyProjectionContractsMatch(first, changed));
}

TEST(HeightAwareOccupancyGrid, ProjectsObstacleHitsAndGroundFreeSpace)
{
  auto scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  scan->push_back(pcl::PointXYZI{1.0F, 0.0F, 0.20F, 1.0F});
  scan->push_back(pcl::PointXYZI{2.0F, 0.0F, 0.00F, 1.0F});
  scan->push_back(pcl::PointXYZI{3.0F, 0.0F, 0.60F, 1.0F});
  GlobalKeyframe keyframe;
  keyframe.id = 0U;
  keyframe.registration_scan = scan;
  keyframe.occupancy_scan = scan;
  HeightAwareOccupancyGrid grid(makeParameters());
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

TEST(HeightAwareOccupancyGrid, DropsReturnsBeyondTheProjectionRange)
{
  // The far return is what a grazing look at a distant wall produces: one
  // sample, poorly localised. Integrating it is worse than ignoring it,
  // because the ray also clears everything between here and there.
  auto scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  scan->push_back(pcl::PointXYZI{2.0F, 0.0F, 0.20F, 1.0F});
  scan->push_back(pcl::PointXYZI{9.0F, 0.0F, 0.20F, 1.0F});
  GlobalKeyframe keyframe;
  keyframe.id = 0U;
  keyframe.registration_scan = scan;
  keyframe.occupancy_scan = scan;
  const auto parameters = makeParameters(8.0, 25, 65);
  HeightAwareOccupancyGrid grid(parameters);
  grid.begin({keyframe}, {Eigen::Isometry3d::Identity()});
  ASSERT_TRUE(grid.processBatch());

  const auto snapshot = grid.snapshot();
  // The near return is inside the range and occupies its cell; the far one
  // never reaches the grid, so the map simply does not extend that far.
  EXPECT_LT(
    snapshot.origin_cell_x + static_cast<int>(snapshot.width), 9.0 / 0.05);
}

TEST(HeightAwareOccupancyGrid, UsesDenseOccupancyScanWithoutClosingDoorways)
{
  auto registration_scan =
    std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  registration_scan->push_back(pcl::PointXYZI{2.0F, -0.20F, 0.20F, 1.0F});
  auto occupancy_scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (const float y : {-0.20F, -0.15F, -0.10F, 0.10F, 0.15F, 0.20F}) {
    occupancy_scan->push_back(pcl::PointXYZI{2.0F, y, 0.20F, 1.0F});
  }
  GlobalKeyframe keyframe;
  keyframe.registration_scan = registration_scan;
  keyframe.occupancy_scan = occupancy_scan;
  HeightAwareOccupancyGrid grid(makeParameters());
  grid.begin({keyframe}, {Eigen::Isometry3d::Identity()});
  ASSERT_TRUE(grid.processBatch());

  const auto snapshot = grid.snapshot();
  const auto value = [&](int x, int y) {
      return snapshot.data[
        static_cast<std::size_t>(y - snapshot.origin_cell_y) * snapshot.width +
        static_cast<std::size_t>(x - snapshot.origin_cell_x)];
    };
  std::size_t occupied_wall_cells = 0U;
  for (int y = snapshot.origin_cell_y;
    y < snapshot.origin_cell_y + static_cast<int>(snapshot.height); ++y)
  {
    if (value(40, y) >= 50) {
      ++occupied_wall_cells;
    }
  }
  EXPECT_GE(occupied_wall_cells, 4U);
  EXPECT_LT(value(40, 0), 50);
}

TEST(HeightAwareOccupancyGrid, RejectsANonPositiveProjectionRange)
{
  const auto parameters = makeParameters(0.0, 25, 65);
  EXPECT_THROW(HeightAwareOccupancyGrid{parameters}, std::invalid_argument);
}

TEST(HeightAwareOccupancyGrid, AppendsNewKeyframesWithoutReplayingHistory)
{
  auto first_scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  first_scan->push_back(pcl::PointXYZI{1.0F, 0.0F, 0.20F, 1.0F});
  GlobalKeyframe first;
  first.id = 0U;
  first.registration_scan = first_scan;
  first.occupancy_scan = first_scan;
  HeightAwareOccupancyGrid grid(makeParameters());
  grid.begin({first}, {Eigen::Isometry3d::Identity()});
  ASSERT_TRUE(grid.processBatch());

  auto second_scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  second_scan->push_back(pcl::PointXYZI{2.0F, 0.0F, 0.20F, 1.0F});
  GlobalKeyframe second;
  second.id = 1U;
  second.registration_scan = second_scan;
  second.occupancy_scan = second_scan;
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
  ground.registration_scan = ground_scan;
  ground.occupancy_scan = ground_scan;
  HeightAwareOccupancyGrid grid(makeParameters(100.0, 25, 50));
  grid.begin({ground}, {Eigen::Isometry3d::Identity()});
  ASSERT_TRUE(grid.processBatch());

  const auto cell = [](const auto & snapshot, int x) {
      return snapshot.data[static_cast<std::size_t>(-snapshot.origin_cell_y) * snapshot.width +
               static_cast<std::size_t>(x - snapshot.origin_cell_x)];
    };
  EXPECT_EQ(cell(grid.snapshot(), 20), 40);
  EXPECT_EQ(cell(grid.navigationSnapshot(grid.snapshot()), 20), -1);

  // Four clear observations are required before a cell becomes navigation
  // free. Repeating the immutable keyframe here models later views along the
  // same ray without changing the integration contract.
  grid.append(ground, Eigen::Isometry3d::Identity());
  grid.append(ground, Eigen::Isometry3d::Identity());
  grid.append(ground, Eigen::Isometry3d::Identity());
  EXPECT_EQ(cell(grid.navigationSnapshot(grid.snapshot()), 20), 0);

  auto obstacle_scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  obstacle_scan->push_back(pcl::PointXYZI{2.0F, 0.0F, 0.20F, 1.0F});
  GlobalKeyframe obstacle;
  obstacle.id = 1U;
  obstacle.registration_scan = obstacle_scan;
  obstacle.occupancy_scan = obstacle_scan;
  HeightAwareOccupancyGrid obstacle_grid(makeParameters(100.0, 25, 50));
  obstacle_grid.begin({obstacle}, {Eigen::Isometry3d::Identity()});
  ASSERT_TRUE(obstacle_grid.processBatch());
  EXPECT_EQ(cell(obstacle_grid.navigationSnapshot(obstacle_grid.snapshot()), 40), 100);

  // A later contradictory ground ray weakens the obstacle posterior to 61.
  // The evidence still net-favours an obstacle, so navigation must see one:
  // publishing unknown here is what let exploration target the wall.
  auto long_ground_scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  long_ground_scan->push_back(pcl::PointXYZI{2.0F, 0.0F, 0.00F, 1.0F});
  GlobalKeyframe long_ground;
  long_ground.id = 2U;
  long_ground.registration_scan = long_ground_scan;
  long_ground.occupancy_scan = long_ground_scan;
  obstacle_grid.append(long_ground, Eigen::Isometry3d::Identity());
  EXPECT_EQ(cell(obstacle_grid.snapshot(), 40), 61);
  EXPECT_EQ(cell(obstacle_grid.navigationSnapshot(obstacle_grid.snapshot()), 40), 100);
}

TEST(HeightAwareOccupancyGrid, EvidenceLeaningOccupiedIsNotPublishedAsUnexplored)
{
  // The three cases the navigation map must keep apart. Only the middle one
  // is genuinely "go and look": the leaning-occupied cell is a wall seen a
  // few times, and calling it unknown is what turned walls into frontiers.
  slam_robot_slam::OccupancyGridSnapshot snapshot;
  snapshot.resolution = 0.05;
  snapshot.width = 4U;
  snapshot.height = 1U;
  snapshot.data = {-1, 23, 40, 51};

  const auto navigation =
    HeightAwareOccupancyGrid(makeParameters(100.0, 25, 50))
    .navigationSnapshot(snapshot);

  EXPECT_EQ(navigation.data[0], -1);   // never observed
  EXPECT_EQ(navigation.data[1], 0);    // three clear rays
  EXPECT_EQ(navigation.data[2], -1);   // one clear ray, still worth visiting
  EXPECT_EQ(navigation.data[3], 100);  // evidence net-favours an obstacle
}

TEST(HeightAwareOccupancyGrid, UsesConfiguredFreeEvidenceThreshold)
{
  auto scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  scan->push_back(pcl::PointXYZI{1.0F, 0.0F, 0.00F, 1.0F});
  GlobalKeyframe keyframe;
  keyframe.id = 0U;
  keyframe.registration_scan = scan;
  keyframe.occupancy_scan = scan;
  HeightAwareOccupancyGrid grid(makeParameters(100.0, 25, 65));
  grid.begin({keyframe}, {Eigen::Isometry3d::Identity()});
  ASSERT_TRUE(grid.processBatch());
  grid.append(keyframe, Eigen::Isometry3d::Identity());
  grid.append(keyframe, Eigen::Isometry3d::Identity());

  const auto cell = [](const auto & snapshot) {
      return snapshot.data[static_cast<std::size_t>(-snapshot.origin_cell_y) * snapshot.width +
               static_cast<std::size_t>(20 - snapshot.origin_cell_x)];
    };
  EXPECT_EQ(cell(grid.snapshot()), 23);
  EXPECT_EQ(cell(grid.navigationSnapshot(grid.snapshot())), 0);
}
}  // namespace slam_robot_slam_3d
