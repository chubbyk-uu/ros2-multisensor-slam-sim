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
  // Circumscribed radius of the Nav2 footprint polygon, rounded up. The
  // polygon's farthest vertex is at hypot(0.295, 0.160) = 0.33560 m, so the
  // 0.335 this used to hold was 0.6 mm *inside* the robot. The 0.15 m margin
  // covered it and no measured spawn was ever unsafe, but a robot geometry
  // that under-reports itself is the wrong direction to be wrong in, and
  // test_footprint_contract.py now keeps the two in step.
  //
  // Kept separate from the margin so it stays obvious which part is the robot
  // and which part is deliberately conservative clearance. It has no
  // command-line flag on purpose: it is a contract with the robot model, and
  // shrinking it to fit a world would quietly undo the guarantee.
  double robot_circumscribed_radius{0.336};
  double safety_margin{0.15};
  double robot_height{0.35};
  double vertical_margin{0.10};
  double reference_x{0.0};
  double reference_y{0.0};
  double spawn_z{0.03};
  double minimum_spawn_separation{2.0};
  // Applied on top of the ordinary clearance to collision geometry belonging
  // to models Gazebo is free to move. Their SDF pose is exact at t=0, but the
  // sampler runs before the simulator starts and the robot is spawned later
  // still, with `gz sim -r` running in between, so a crate can have settled or
  // slid by the time it matters. This buys room for that without pretending to
  // predict it.
  double non_static_extra_margin{0.25};
  std::size_t maximum_grid_cells{4000000U};
};

struct WorldParsingPolicy
{
  // Refuse a world containing collision geometry that may move, instead of
  // avoiding it conservatively. Off by default: refusing makes the sampler
  // unusable on any world holding a single movable prop, which pushes the
  // operator back to hand-picked coordinates -- a worse safety outcome than
  // a conservative envelope. Formal campaigns can turn it on.
  bool reject_non_static{false};
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
  // From a model Gazebo may move, so its pose is only true at t=0.
  bool dynamic{false};
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
  // Reported rather than silently absorbed, so a record says whether the world
  // held anything the sampler could only avoid approximately.
  std::size_t non_static_collisions{0U};
};

class SdfWorldGeometryParser
{
public:
  SdfWorldGeometryParser() = default;
  explicit SdfWorldGeometryParser(WorldParsingPolicy policy)
  : policy_(policy) {}

  ParsedWorldGeometry parse(const std::string & world_path) const;

private:
  WorldParsingPolicy policy_;
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

// Version 1 was a hand-concatenated object carrying only the world, the seed,
// the resolution, the safe-cell count and the poses. It could not be replayed:
// every other value that moves a pose -- the robot envelope, the separation,
// the reference point -- was left to whatever the caller happened to pass.
// Anything that changes which fields exist bumps this again.
// Version 2 added the parameters and the sampling bounds. Version 3 added the
// non-static policy and the parsed-geometry counts: without them a record
// cannot say whether the world held anything that might have moved, or which
// of the two ways the sampler was told to treat it.
constexpr int kSpawnRecordSchemaVersion = 3;

// Returns the record as one line of JSON, so a caller that kept only a log can
// still recover what produced these poses. Built here rather than in main so a
// test can pin the field set: a record that silently loses a field is worse
// than no record, because it still looks replayable.
std::string formatSpawnRecord(
  const std::string & world_path,
  const ParsedWorldGeometry & geometry,
  const SafeSpawnGrid & grid,
  const SpawnSamplingParameters & parameters,
  const WorldParsingPolicy & policy,
  std::uint64_t seed,
  const std::vector<SpawnPose> & poses);

}  // namespace slam_robot_gazebo
