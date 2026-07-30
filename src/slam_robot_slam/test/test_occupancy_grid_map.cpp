#include <gtest/gtest.h>

#include <stdexcept>

#include "slam_robot_slam/occupancy_grid_map.hpp"

namespace slam_robot_slam
{
namespace
{

int8_t cellAt(
  const OccupancyGridSnapshot & map,
  const int cell_x,
  const int cell_y)
{
  const auto column =
    static_cast<std::size_t>(cell_x - map.origin_cell_x);
  const auto row =
    static_cast<std::size_t>(cell_y - map.origin_cell_y);
  return map.data.at(row * map.width + column);
}

TEST(OccupancyGridMap, MarksRayFreeAndEndpointOccupied)
{
  OccupancyGridMapParameters parameters;
  parameters.resolution = 0.5;
  parameters.padding_cells = 0;
  OccupancyGridMap map(parameters);

  map.updateRay(Point2D{0.1F, 0.1F}, Point2D{1.1F, 0.1F}, true);
  const auto snapshot = map.snapshot();

  EXPECT_EQ(snapshot.width, 3U);
  EXPECT_EQ(snapshot.height, 1U);
  EXPECT_LT(cellAt(snapshot, 0, 0), 50);
  EXPECT_LT(cellAt(snapshot, 1, 0), 50);
  EXPECT_GT(cellAt(snapshot, 2, 0), 50);
}

TEST(OccupancyGridMap, TreatsNoReturnEndpointAsFree)
{
  OccupancyGridMapParameters parameters;
  parameters.resolution = 1.0;
  parameters.padding_cells = 0;
  OccupancyGridMap map(parameters);

  map.updateRay(Point2D{0.0F, 0.0F}, Point2D{0.0F, 2.0F}, false);
  const auto snapshot = map.snapshot();

  EXPECT_LT(cellAt(snapshot, 0, 0), 50);
  EXPECT_LT(cellAt(snapshot, 0, 1), 50);
  EXPECT_LT(cellAt(snapshot, 0, 2), 50);
}

TEST(OccupancyGridMap, PreservesUnknownCellsInsideDynamicBounds)
{
  OccupancyGridMapParameters parameters;
  parameters.resolution = 1.0;
  parameters.padding_cells = 0;
  OccupancyGridMap map(parameters);

  map.updateRay(Point2D{-1.0F, -1.0F}, Point2D{-1.0F, 1.0F}, true);
  map.updateRay(Point2D{1.0F, -1.0F}, Point2D{1.0F, 1.0F}, true);
  const auto snapshot = map.snapshot();

  EXPECT_EQ(cellAt(snapshot, 0, 0), -1);
  EXPECT_EQ(map.observedCellCount(), 6U);
}

TEST(OccupancyGridMap, RejectsInvalidProbabilities)
{
  OccupancyGridMapParameters parameters;
  parameters.hit_probability = 0.4;
  EXPECT_THROW(OccupancyGridMap map(parameters), std::invalid_argument);
}

}  // namespace
}  // namespace slam_robot_slam
