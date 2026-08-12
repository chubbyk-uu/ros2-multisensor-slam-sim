// Copyright 2026 Jerry

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace slam_robot_gazebo
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Bounds2D
{
  double minimum_x{0.0};
  double minimum_y{0.0};
  double maximum_x{0.0};
  double maximum_y{0.0};
};

struct SpawnPose
{
  double x{0.0};
  double y{0.0};
  double z{0.03};
  double yaw{0.0};
};

struct SpawnSamplingParameters
{
  double resolution{0.05};
  // Circumscribed radius of the Nav2 footprint. Keeping this separate from
  // the margin makes it clear which part is robot geometry and which part is
  // deliberately conservative spawn clearance.
  double robot_circumscribed_radius{0.335};
  double safety_margin{0.15};
  double robot_height{0.35};
  double vertical_margin{0.10};
  double reference_x{0.0};
  double reference_y{0.0};
  double spawn_z{0.03};
  double minimum_spawn_separation{2.0};
  std::size_t maximum_grid_cells{4000000U};
};

enum class FootprintType
{
  kRectangle,
  kCircle,
};

struct CollisionFootprint
{
  FootprintType type{FootprintType::kRectangle};
  std::string name;
  Point2D center;
  double yaw{0.0};
  double size_x{0.0};
  double size_y{0.0};
  double radius{0.0};
  double minimum_z{0.0};
  double maximum_z{0.0};
};

struct SupportSurface
{
  Point2D center;
  double yaw{0.0};
  double size_x{0.0};
  double size_y{0.0};
  double height{0.0};
};

struct ParsedWorldGeometry
{
  std::string world_name;
  std::vector<SupportSurface> supports;
  std::vector<CollisionFootprint> obstacles;
  Bounds2D sampling_bounds;
};

class SdfWorldGeometryParser
{
public:
  ParsedWorldGeometry parse(const std::string & world_path) const;
};

class SafeSpawnGrid
{
public:
  SafeSpawnGrid(
    ParsedWorldGeometry geometry,
    SpawnSamplingParameters parameters = SpawnSamplingParameters{});

  std::vector<SpawnPose> sample(std::size_t count, std::uint64_t seed) const;
  bool isSafe(double x, double y) const;
  std::size_t safeCellCount() const;
  const Bounds2D & bounds() const;
  double resolution() const;
  void writeDebugPgm(
    const std::string & path,
    const std::vector<SpawnPose> & samples = {}) const;

private:
  std::size_t index(std::size_t x, std::size_t y) const;
  Point2D cellCenter(std::size_t index) const;

  ParsedWorldGeometry geometry_;
  SpawnSamplingParameters parameters_;
  std::size_t width_{0U};
  std::size_t height_{0U};
  std::vector<std::uint8_t> safe_cells_;
};

std::uint64_t makeSpawnSeed();

}  // namespace slam_robot_gazebo
