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
