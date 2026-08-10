// Copyright 2026 Jerry

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "slam_robot_navigation/frontier_detector.hpp"

namespace
{

nav_msgs::msg::OccupancyGrid makeMap()
{
  nav_msgs::msg::OccupancyGrid map;
  map.info.resolution = 0.1F;
  map.info.width = 30;
  map.info.height = 20;
  map.data.assign(map.info.width * map.info.height, -1);
  for (std::size_t y = 4; y < 16; ++y) {
    for (std::size_t x = 4; x < 25; ++x) {
      map.data[y * map.info.width + x] = 0;
    }
  }
  for (std::size_t y = 7; y < 13; ++y) {
    map.data[y * map.info.width + 14] = 100;
  }
  return map;
}

nav_msgs::msg::OccupancyGrid makeSeparatedFrontiersMap()
{
  nav_msgs::msg::OccupancyGrid map;
  map.info.resolution = 0.1F;
  map.info.width = 140;
  map.info.height = 60;
  map.data.assign(map.info.width * map.info.height, -1);
  const auto fill_free = [&map](
    std::size_t min_x, std::size_t max_x,
    std::size_t min_y, std::size_t max_y) {
      for (std::size_t y = min_y; y < max_y; ++y) {
        for (std::size_t x = min_x; x < max_x; ++x) {
          map.data[y * map.info.width + x] = 0;
        }
      }
    };
  fill_free(5U, 10U, 25U, 30U);
  fill_free(80U, 120U, 10U, 50U);
  return map;
}

TEST(FrontierDetector, FindsAndRanksSafeFrontierClusters)
{
  slam_robot_navigation::FrontierDetectorParameters parameters;
  parameters.minimum_cluster_cells = 4;
  parameters.minimum_clearance = 0.2;
  slam_robot_navigation::FrontierDetector detector(parameters);
  const auto candidates = detector.detect(makeMap(), 1.0, 1.0);

  ASSERT_FALSE(candidates.empty());
  for (const auto & candidate : candidates) {
    EXPECT_GE(candidate.cluster_cells, 4U);
    EXPECT_GE(candidate.clearance, 0.2);
    EXPECT_TRUE(std::isfinite(candidate.yaw));
    EXPECT_TRUE(std::isfinite(candidate.score));
    EXPECT_GE(candidate.cell_x, 4U);
    EXPECT_LE(candidate.cell_x, 24U);
  }
}

TEST(FrontierDetector, LargerFrontierOutranksTinyNearbyFragment)
{
  slam_robot_navigation::FrontierDetectorParameters parameters;
  parameters.minimum_cluster_cells = 1U;
  parameters.minimum_clearance = 0.0;
  slam_robot_navigation::FrontierDetector detector(parameters);
  const auto candidates = detector.detect(makeSeparatedFrontiersMap(), 0.7, 2.7);

  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_GT(candidates.front().cluster_cells, candidates.back().cluster_cells);
  EXPECT_GT(candidates.front().robot_distance, candidates.back().robot_distance);
  EXPECT_GT(candidates.front().score, candidates.back().score);
}

TEST(FrontierDetector, DoesNotGuessProbabilisticCellsAsUnknown)
{
  // The map producer, not this generic detector, owns probability thresholds.
  // A raw value of 40 can mean a single free ray or a weakened obstacle hit;
  // treating it as unknown here would make those meanings indistinguishable.
  auto map = makeMap();
  for (std::size_t y = 4; y < 16; ++y) {
    for (std::size_t x = 4; x < 25; ++x) {
      if (y == 4 || y == 15 || x == 4 || x == 24) {
        map.data[y * map.info.width + x] = 40;
      }
    }
  }
  slam_robot_navigation::FrontierDetectorParameters parameters;
  parameters.minimum_cluster_cells = 4;
  parameters.minimum_clearance = 0.2;
  const auto candidates =
    slam_robot_navigation::FrontierDetector(parameters).detect(map, 1.0, 1.0);

  EXPECT_TRUE(candidates.empty());
}

TEST(FrontierDetector, OccupiedNeighboursAreNotAFrontier)
{
  // Widening the boundary test must not turn walls into exploration targets.
  auto map = makeMap();
  for (std::size_t y = 0; y < map.info.height; ++y) {
    for (std::size_t x = 0; x < map.info.width; ++x) {
      auto & cell = map.data[y * map.info.width + x];
      if (cell < 0) {
        cell = 100;
      }
    }
  }
  slam_robot_navigation::FrontierDetectorParameters parameters;
  parameters.minimum_cluster_cells = 1;
  parameters.minimum_clearance = 0.0;
  const auto candidates =
    slam_robot_navigation::FrontierDetector(parameters).detect(map, 1.0, 1.0);

  EXPECT_TRUE(candidates.empty());
}

TEST(FrontierDetector, ExposesItsConfiguredFreeThreshold)
{
  slam_robot_navigation::FrontierDetectorParameters parameters;
  parameters.free_maximum = 15;
  EXPECT_EQ(slam_robot_navigation::FrontierDetector(parameters).freeMaximum(), 15);
}

TEST(FrontierDetector, RejectsClustersWithoutObstacleClearance)
{
  auto map = makeMap();
  slam_robot_navigation::FrontierDetectorParameters parameters;
  parameters.minimum_cluster_cells = 4;
  parameters.minimum_clearance = 10.0;
  slam_robot_navigation::FrontierDetector detector(parameters);
  EXPECT_TRUE(detector.detect(map, 1.0, 1.0).empty());
}

TEST(FrontierDetector, RejectsInvalidInputsAndParameters)
{
  auto parameters = slam_robot_navigation::FrontierDetectorParameters{};
  parameters.minimum_cluster_cells = 0;
  EXPECT_THROW(
    {slam_robot_navigation::FrontierDetector detector(parameters);},
    std::invalid_argument);

  auto map = makeMap();
  map.data.pop_back();
  EXPECT_THROW(
    slam_robot_navigation::FrontierDetector({}).detect(map, 0.0, 0.0),
    std::invalid_argument);
}

}  // namespace
