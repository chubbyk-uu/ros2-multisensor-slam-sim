// Copyright 2026 Jerry

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include "slam_robot_gazebo/safe_spawn_sampler.hpp"

namespace slam_robot_gazebo
{
namespace
{

ParsedWorldGeometry simpleWorld()
{
  ParsedWorldGeometry geometry;
  geometry.world_name = "test";
  geometry.supports.push_back({{0.0, 0.0}, 0.0, 10.0, 10.0, 0.0});
  geometry.sampling_bounds = {-5.0, -5.0, 5.0, 5.0};
  return geometry;
}

TEST(SafeSpawnGrid, RejectsAnObstacleThatOverlapsTheCompleteRobotHeight)
{
  auto geometry = simpleWorld();
  geometry.obstacles.push_back({
        FootprintType::kRectangle, "low_header", {2.0, 0.0}, 0.0, 1.0, 1.0,
        0.0, 0.40, 0.60});
  SpawnSamplingParameters parameters;
  parameters.resolution = 0.05;
  SafeSpawnGrid grid(std::move(geometry), parameters);

  EXPECT_FALSE(grid.isSafe(2.0, 0.0));
  EXPECT_FALSE(grid.isSafe(1.55, 0.0));
}

TEST(SafeSpawnGrid, AllowsAHeaderAboveTheSweptHeight)
{
  auto geometry = simpleWorld();
  geometry.obstacles.push_back({
        FootprintType::kRectangle, "high_header", {2.0, 0.0}, 0.0, 1.0, 1.0,
        0.0, 0.55, 0.75});
  SafeSpawnGrid grid(std::move(geometry));

  EXPECT_TRUE(grid.isSafe(2.0, 0.0));
}

TEST(SafeSpawnGrid, IncludesTheGazeboSpawnLiftInVerticalClearance)
{
  auto geometry = simpleWorld();
  geometry.obstacles.push_back({
        FootprintType::kRectangle, "spawn_height_header", {2.0, 0.0}, 0.0, 1.0, 1.0,
        0.0, 0.46, 0.47});
  SafeSpawnGrid grid(std::move(geometry));

  EXPECT_FALSE(grid.isSafe(2.0, 0.0));
}

TEST(SafeSpawnGrid, KeepsOnlyTheReferenceConnectedComponent)
{
  auto geometry = simpleWorld();
  geometry.obstacles.push_back({
        FootprintType::kRectangle, "divider", {1.0, 0.0}, 0.0, 0.2, 10.0,
        0.0, 0.0, 1.0});
  SafeSpawnGrid grid(std::move(geometry));

  EXPECT_TRUE(grid.isSafe(0.0, 0.0));
  EXPECT_FALSE(grid.isSafe(3.0, 0.0));
}

TEST(SafeSpawnGrid, FixedSeedReproducesSeparatedSamples)
{
  SafeSpawnGrid grid(simpleWorld());

  const auto first = grid.sample(5U, 1234U);
  const auto second = grid.sample(5U, 1234U);

  ASSERT_EQ(first.size(), second.size());
  for (std::size_t index = 0U; index < first.size(); ++index) {
    EXPECT_DOUBLE_EQ(first[index].x, second[index].x);
    EXPECT_DOUBLE_EQ(first[index].y, second[index].y);
    EXPECT_DOUBLE_EQ(first[index].yaw, second[index].yaw);
    EXPECT_TRUE(grid.isSafe(first[index].x, first[index].y));
    for (std::size_t other = 0U; other < index; ++other) {
      EXPECT_GE(
        std::hypot(first[index].x - first[other].x, first[index].y - first[other].y),
        2.0);
    }
  }
}

TEST(SdfWorldGeometryParser, ParsesEveryProjectAcceptanceWorld)
{
  const std::filesystem::path directory(TEST_WORLD_DIRECTORY);
  for (const auto & name : {"slam_world.sdf", "structured_loop_3d.sdf", "large_warehouse.sdf"}) {
    SCOPED_TRACE(name);
    const auto geometry = SdfWorldGeometryParser{}.parse((directory / name).string());
    EXPECT_FALSE(geometry.world_name.empty());
    EXPECT_FALSE(geometry.supports.empty());
    EXPECT_FALSE(geometry.obstacles.empty());
    const SafeSpawnGrid grid(geometry);
    EXPECT_GT(grid.safeCellCount(), 1000U);
    EXPECT_TRUE(grid.isSafe(0.0, 0.0));
    EXPECT_EQ(grid.sample(5U, 20260812U).size(), 5U);
  }
}

TEST(SafeSpawnGrid, RejectsInvalidParametersAndUnsafeReference)
{
  auto invalid = SpawnSamplingParameters{};
  invalid.robot_height = 0.0;
  EXPECT_THROW(SafeSpawnGrid(simpleWorld(), invalid), std::invalid_argument);

  auto geometry = simpleWorld();
  geometry.obstacles.push_back({
        FootprintType::kCircle, "origin", {0.0, 0.0}, 0.0, 0.0, 0.0,
        1.0, 0.0, 1.0});
  EXPECT_THROW(SafeSpawnGrid(std::move(geometry)), std::runtime_error);
}

}  // namespace
}  // namespace slam_robot_gazebo
