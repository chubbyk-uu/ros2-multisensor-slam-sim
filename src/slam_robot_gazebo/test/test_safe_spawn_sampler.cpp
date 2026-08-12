// Copyright 2026 Jerry

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "slam_robot_gazebo/safe_spawn_sampler.hpp"

namespace slam_robot_gazebo
{
namespace
{

ParsedWorldGeometry simpleWorld()
{
  ParsedWorldGeometry geometry;
  geometry.world_name = "test";
  geometry.support_candidates.push_back(
    {{{0.0, 0.0}, 0.0, 10.0, 10.0, 0.0},
      {FootprintType::kRectangle, "ground", {0.0, 0.0}, 0.0, 10.0, 10.0, 0.0, 0.0, 0.0, false}});
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
    EXPECT_FALSE(geometry.support_candidates.empty());
    EXPECT_FALSE(geometry.obstacles.empty());
    const SafeSpawnGrid grid(geometry);
    EXPECT_GT(grid.safeCellCount(), 1000U);
    EXPECT_TRUE(grid.isSafe(0.0, 0.0));
    EXPECT_EQ(grid.sample(5U, 20260812U).size(), 5U);
  }
}

TEST(SdfWorldGeometryParser, AMovableCrateIsAvoidedRatherThanIgnored)
{
  const std::filesystem::path world =
    std::filesystem::path(TEST_FIXTURE_WORLD_DIRECTORY) / "non_static_prop.sdf";

  const auto geometry = SdfWorldGeometryParser{}.parse(world.string());

  // It used to be skipped outright, which made a crate that is simply not
  // marked static invisible to the sampler and legal to spawn into.
  EXPECT_EQ(geometry.non_static_collisions, 1U);
  ASSERT_EQ(geometry.obstacles.size(), 1U);
  EXPECT_TRUE(geometry.obstacles.front().dynamic);

  SpawnSamplingParameters parameters;
  const SafeSpawnGrid grid(geometry, parameters);
  EXPECT_FALSE(grid.isSafe(3.0, 0.0));
  // Beyond the crate's own half width plus the ordinary envelope, but inside
  // the extra room a movable body gets, because its pose is only true at t=0.
  EXPECT_FALSE(grid.isSafe(3.0 + 0.25 + 0.486 + 0.10, 0.0));
  EXPECT_TRUE(grid.isSafe(3.0 + 0.25 + 0.486 + 0.30, 0.0));
}

TEST(SdfWorldGeometryParser, StrictModeRefusesAWorldItCanOnlyAvoidApproximately)
{
  const std::filesystem::path world =
    std::filesystem::path(TEST_FIXTURE_WORLD_DIRECTORY) / "non_static_prop.sdf";
  WorldParsingPolicy policy;
  policy.reject_non_static = true;

  EXPECT_THROW(
    SdfWorldGeometryParser{policy}.parse(world.string()), std::runtime_error);
}

TEST(SdfWorldGeometryParser, EveryAcceptanceWorldIsFullyStatic)
{
  // The reason strict mode is not the default is that it would cost nothing
  // here and everything on a world with one movable prop. If this ever fails,
  // the campaigns have started measuring a world the sampler can only
  // approximate, and that belongs in the record rather than in a surprise.
  const std::filesystem::path directory(TEST_WORLD_DIRECTORY);
  WorldParsingPolicy policy;
  policy.reject_non_static = true;
  for (const auto & name : {"slam_world.sdf", "structured_loop_3d.sdf", "large_warehouse.sdf"}) {
    SCOPED_TRACE(name);
    EXPECT_NO_THROW(
      SdfWorldGeometryParser{policy}.parse((directory / name).string()));
  }
}

SupportCandidate flatSlab(
  const std::string & name, Point2D center, double size_x, double size_y,
  double top, double thickness)
{
  SupportSurface surface{center, 0.0, size_x, size_y, top};
  CollisionFootprint body{
    FootprintType::kRectangle, name, center, 0.0, size_x, size_y, 0.0,
    top - thickness, top, false};
  return {surface, body};
}

TEST(SafeSpawnGrid, ALowKerbIsAnObstacleRatherThanFloor)
{
  // The failure the absolute height band allowed: a 0.08 m threshold sits
  // inside -0.10..0.10, so it was read as ground, removed from the obstacle
  // list, and the robot could be created straddling it.
  auto geometry = simpleWorld();
  geometry.support_candidates.push_back(
    flatSlab("kerb", {2.0, 0.0}, 1.0, 1.0, 0.08, 0.08));
  SafeSpawnGrid grid(std::move(geometry));

  EXPECT_DOUBLE_EQ(grid.referenceHeight(), 0.0);
  EXPECT_EQ(grid.demotedSupportCount(), 1U);
  EXPECT_FALSE(grid.isSafe(2.0, 0.0));
  // And it is inflated by the full footprint, not merely un-drivable: the
  // robot must not overlap it either.
  EXPECT_FALSE(grid.isSafe(2.0 + 0.5 + 0.40, 0.0));
  EXPECT_TRUE(grid.isSafe(2.0 + 0.5 + 0.60, 0.0));
}

TEST(SafeSpawnGrid, FlushSlabsAreOneFloor)
{
  // A floor laid in pieces is still a floor, which is why the rule is a step
  // height rather than "only the slab under the reference point".
  auto geometry = simpleWorld();
  geometry.support_candidates.push_back(
    flatSlab("annex", {9.0, 0.0}, 8.0, 4.0, 0.0, 0.05));
  geometry.sampling_bounds = {-5.0, -5.0, 13.0, 5.0};
  SafeSpawnGrid grid(std::move(geometry));

  EXPECT_EQ(grid.demotedSupportCount(), 0U);
  EXPECT_TRUE(grid.isSafe(0.0, 0.0));
  EXPECT_TRUE(grid.isSafe(9.0, 0.0));
}

TEST(SafeSpawnGrid, AHeaderOverheadStaysDrivableUnder)
{
  // Demoting a candidate does not by itself block anything: the vertical test
  // still decides. This is the structured world's doorway header at 0.70 m.
  auto geometry = simpleWorld();
  geometry.support_candidates.push_back(
    flatSlab("header", {2.0, 0.0}, 2.8, 0.3, 0.70, 0.15));
  SafeSpawnGrid grid(std::move(geometry));

  EXPECT_EQ(grid.demotedSupportCount(), 1U);
  EXPECT_TRUE(grid.isSafe(2.0, 0.0));
}

TEST(SafeSpawnGrid, MultiLevelSupportUnderTheReferencePointIsRefused)
{
  // Silently taking the highest would make the ground beneath it an obstacle
  // and leave almost nothing safe, which reads as a broken world rather than
  // an unsupported one. Refusing says which it is.
  auto geometry = simpleWorld();
  geometry.support_candidates.push_back(
    flatSlab("platform", {0.0, 0.0}, 4.0, 4.0, 0.50, 0.10));

  EXPECT_THROW(SafeSpawnGrid(std::move(geometry)), std::runtime_error);
}

TEST(SafeSpawnGrid, AReferencePointOffEveryFloorIsRefusedWithItsOwnReason)
{
  auto geometry = simpleWorld();
  SpawnSamplingParameters parameters;
  parameters.reference_x = 20.0;
  parameters.reference_y = 20.0;
  geometry.sampling_bounds = {-5.0, -5.0, 25.0, 25.0};

  EXPECT_THROW(SafeSpawnGrid(std::move(geometry), parameters), std::runtime_error);
}

TEST(SpawnRecord, CarriesEveryValueThatMovesAPose)
{
  SpawnSamplingParameters parameters;
  parameters.resolution = 0.05;
  const auto geometry = simpleWorld();
  const SafeSpawnGrid grid(geometry, parameters);
  const auto poses = grid.sample(2U, 4242U);

  const auto record = nlohmann::json::parse(
    formatSpawnRecord(
      "/tmp/w.sdf", geometry, grid, parameters, WorldParsingPolicy{}, 4242U, poses));

  // Pinned as a set, not spot-checked. A record that silently drops a field
  // still parses and still looks replayable, so the failure would only surface
  // when someone tried to reproduce a batch and could not say why it differed.
  EXPECT_EQ(
    record.at("parameters").get<nlohmann::json::object_t>().size(), 11U);
  for (const auto & key : {"resolution", "robot_circumscribed_radius",
      "safety_margin", "robot_height", "vertical_margin",
      "minimum_spawn_separation", "spawn_z", "reference_x", "reference_y",
      "non_static_extra_margin", "maximum_grid_cells"})
  {
    EXPECT_TRUE(record.at("parameters").contains(key)) << key;
  }
  for (const auto & key : {"schema_version", "world", "world_name", "seed",
      "parameters", "world_parsing_policy", "world_geometry", "support_resolution",
      "sampling_bounds", "safe_cells", "poses"})
  {
    EXPECT_TRUE(record.contains(key)) << key;
  }
  EXPECT_EQ(record.at("schema_version").get<int>(), 3);
  EXPECT_EQ(record.at("seed").get<std::uint64_t>(), 4242U);
  EXPECT_EQ(record.at("world").get<std::string>(), "/tmp/w.sdf");
  EXPECT_EQ(record.at("sampling_bounds").at("minimum_x").get<double>(), -5.0);
  ASSERT_EQ(record.at("poses").size(), 2U);
  // Full precision, not the six places the hand-written version emitted: a
  // truncated pose is not the pose the simulator was given.
  EXPECT_DOUBLE_EQ(record.at("poses")[0].at("yaw").get<double>(), poses[0].yaw);
}

TEST(SpawnRecord, RecordsTheParametersItWasGivenRatherThanTheDefaults)
{
  SpawnSamplingParameters parameters;
  parameters.safety_margin = 0.42;
  parameters.minimum_spawn_separation = 3.5;
  const auto geometry = simpleWorld();
  const SafeSpawnGrid grid(geometry, parameters);

  const auto record = nlohmann::json::parse(
    formatSpawnRecord(
      "/tmp/w.sdf", geometry, grid, parameters, WorldParsingPolicy{}, 7U,
      grid.sample(1U, 7U)));

  EXPECT_DOUBLE_EQ(record.at("parameters").at("safety_margin").get<double>(), 0.42);
  EXPECT_DOUBLE_EQ(
    record.at("parameters").at("minimum_spawn_separation").get<double>(), 3.5);
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
