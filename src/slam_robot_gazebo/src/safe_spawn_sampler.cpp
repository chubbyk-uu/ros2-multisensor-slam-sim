// Copyright 2026 Jerry

#include "slam_robot_gazebo/safe_spawn_sampler.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <random>
#include <stdexcept>
#include <utility>

namespace slam_robot_gazebo
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

bool finiteBounds(const Bounds2D & bounds)
{
  return std::isfinite(bounds.minimum_x) && std::isfinite(bounds.minimum_y) &&
         std::isfinite(bounds.maximum_x) && std::isfinite(bounds.maximum_y) &&
         bounds.minimum_x < bounds.maximum_x && bounds.minimum_y < bounds.maximum_y;
}

Point2D localPoint(const Point2D & point, const Point2D & center, double yaw)
{
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const double dx = point.x - center.x;
  const double dy = point.y - center.y;
  return {cosine * dx + sine * dy, -sine * dx + cosine * dy};
}

bool pointInSupport(const Point2D & point, const SupportSurface & support)
{
  const auto local = localPoint(point, support.center, support.yaw);
  return std::abs(local.x) <= support.size_x * 0.5 &&
         std::abs(local.y) <= support.size_y * 0.5;
}

std::string formatHeight(double value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  return stream.str();
}

Bounds2D surfaceBounds(const SupportSurface & surface)
{
  const double cosine = std::abs(std::cos(surface.yaw));
  const double sine = std::abs(std::sin(surface.yaw));
  const double half_x = (cosine * surface.size_x + sine * surface.size_y) * 0.5;
  const double half_y = (sine * surface.size_x + cosine * surface.size_y) * 0.5;
  return {
    surface.center.x - half_x, surface.center.y - half_y,
    surface.center.x + half_x, surface.center.y + half_y};
}

Bounds2D obstacleBounds(const CollisionFootprint & obstacle)
{
  if (obstacle.type == FootprintType::kCircle) {
    return {
      obstacle.center.x - obstacle.radius, obstacle.center.y - obstacle.radius,
      obstacle.center.x + obstacle.radius, obstacle.center.y + obstacle.radius};
  }
  return surfaceBounds(
    {obstacle.center, obstacle.yaw, obstacle.size_x, obstacle.size_y, 0.0});
}

void merge(Bounds2D & into, const Bounds2D & other, bool & initialized)
{
  if (!initialized) {
    into = other;
    initialized = true;
    return;
  }
  into.minimum_x = std::min(into.minimum_x, other.minimum_x);
  into.minimum_y = std::min(into.minimum_y, other.minimum_y);
  into.maximum_x = std::max(into.maximum_x, other.maximum_x);
  into.maximum_y = std::max(into.maximum_y, other.maximum_y);
}

std::string formatBounds(const Bounds2D & bounds)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << "x " << bounds.minimum_x << ".."
         << bounds.maximum_x << ", y " << bounds.minimum_y << ".." << bounds.maximum_y;
  return stream.str();
}

double distanceToObstacle(const Point2D & point, const CollisionFootprint & obstacle)
{
  if (obstacle.type == FootprintType::kCircle) {
    return std::max(0.0, std::hypot(
      point.x - obstacle.center.x, point.y - obstacle.center.y) - obstacle.radius);
  }
  const auto local = localPoint(point, obstacle.center, obstacle.yaw);
  const double dx = std::max(std::abs(local.x) - obstacle.size_x * 0.5, 0.0);
  const double dy = std::max(std::abs(local.y) - obstacle.size_y * 0.5, 0.0);
  return std::hypot(dx, dy);
}

}  // namespace

SafeSpawnGrid::SafeSpawnGrid(
  ParsedWorldGeometry geometry, SpawnSamplingParameters parameters)
: geometry_(std::move(geometry)), parameters_(parameters)
{
  if (!finiteBounds(geometry_.sampling_bounds)) {
    throw std::invalid_argument("world sampling bounds must be finite and non-empty");
  }
  if (!std::isfinite(parameters_.resolution) || parameters_.resolution <= 0.0 ||
    !std::isfinite(parameters_.robot_circumscribed_radius) ||
    parameters_.robot_circumscribed_radius <= 0.0 ||
    !std::isfinite(parameters_.safety_margin) || parameters_.safety_margin < 0.0 ||
    !std::isfinite(parameters_.robot_height) || parameters_.robot_height <= 0.0 ||
    !std::isfinite(parameters_.vertical_margin) || parameters_.vertical_margin < 0.0 ||
    !std::isfinite(parameters_.spawn_z) || parameters_.spawn_z < 0.0 ||
    !std::isfinite(parameters_.minimum_spawn_separation) ||
    parameters_.minimum_spawn_separation < 0.0 ||
    !std::isfinite(parameters_.non_static_extra_margin) ||
    parameters_.non_static_extra_margin < 0.0 || parameters_.maximum_grid_cells == 0U)
  {
    throw std::invalid_argument("invalid safe-spawn sampling parameters");
  }

  resolveSupportLayer();
  geometry_.sampling_bounds = clipSamplingBounds();

  width_ = static_cast<std::size_t>(std::ceil(
      (geometry_.sampling_bounds.maximum_x - geometry_.sampling_bounds.minimum_x) /
      parameters_.resolution));
  height_ = static_cast<std::size_t>(std::ceil(
      (geometry_.sampling_bounds.maximum_y - geometry_.sampling_bounds.minimum_y) /
      parameters_.resolution));
  if (width_ == 0U || height_ == 0U ||
    width_ > parameters_.maximum_grid_cells / height_)
  {
    std::ostringstream message;
    message << "safe-spawn grid of " << width_ << " x " << height_ << " = "
            << (height_ == 0U ? 0U : width_ * height_) << " cells at "
            << parameters_.resolution << " m exceeds maximum_grid_cells "
            << parameters_.maximum_grid_cells << " over bounds "
            << formatBounds(geometry_.sampling_bounds);
    throw std::overflow_error(message.str());
  }

  safe_cells_.assign(width_ * height_, 0U);
  // Gazebo creates base_footprint at spawn_z before contacts settle it onto
  // the support. Include that transient lift so a pose cannot be declared
  // safe and then clip a header during the first physics step.
  const double swept_height = parameters_.spawn_z + parameters_.robot_height +
    parameters_.vertical_margin;
  const double clearance =
    parameters_.robot_circumscribed_radius + parameters_.safety_margin;
  for (std::size_t cell = 0U; cell < safe_cells_.size(); ++cell) {
    const auto point = cellCenter(cell);
    const bool supported = std::any_of(
      supports_.begin(), supports_.end(),
      [&point](const auto & support) {return pointInSupport(point, support);});
    if (!supported) {continue;}
    const double extra = parameters_.non_static_extra_margin;
    const bool blocked = std::any_of(
      geometry_.obstacles.begin(), geometry_.obstacles.end(),
      [&point, clearance, swept_height, extra](const auto & obstacle) {
        const bool overlaps_height =
        obstacle.maximum_z > 0.0 && obstacle.minimum_z < swept_height;
        // A body that may move gets more room, because its recorded pose is
        // only true at t=0 and the simulator runs before the robot appears.
        const double required = obstacle.dynamic ? clearance + extra : clearance;
        return overlaps_height && distanceToObstacle(point, obstacle) < required;
      });
    safe_cells_[cell] = blocked ? 0U : 1U;
  }

  const auto reference_x = static_cast<std::int64_t>(std::floor(
      (parameters_.reference_x - geometry_.sampling_bounds.minimum_x) /
      parameters_.resolution));
  const auto reference_y = static_cast<std::int64_t>(std::floor(
      (parameters_.reference_y - geometry_.sampling_bounds.minimum_y) /
      parameters_.resolution));
  if (reference_x < 0 || reference_y < 0 ||
    reference_x >= static_cast<std::int64_t>(width_) ||
    reference_y >= static_cast<std::int64_t>(height_) ||
    safe_cells_[index(
      static_cast<std::size_t>(reference_x), static_cast<std::size_t>(reference_y))] == 0U)
  {
    throw std::runtime_error("reference spawn is not inside the safe free-space grid");
  }

  std::vector<std::uint8_t> connected(safe_cells_.size(), 0U);
  std::deque<std::size_t> pending;
  const auto reference = index(
    static_cast<std::size_t>(reference_x), static_cast<std::size_t>(reference_y));
  connected[reference] = 1U;
  pending.push_back(reference);
  constexpr std::int64_t offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  while (!pending.empty()) {
    const auto current = pending.front();
    pending.pop_front();
    const auto x = static_cast<std::int64_t>(current % width_);
    const auto y = static_cast<std::int64_t>(current / width_);
    for (const auto & offset : offsets) {
      const std::int64_t nx = x + offset[0];
      const std::int64_t ny = y + offset[1];
      if (nx < 0 || ny < 0 || nx >= static_cast<std::int64_t>(width_) ||
        ny >= static_cast<std::int64_t>(height_))
      {
        continue;
      }
      const auto neighbour = index(
        static_cast<std::size_t>(nx), static_cast<std::size_t>(ny));
      if (safe_cells_[neighbour] != 0U && connected[neighbour] == 0U) {
        connected[neighbour] = 1U;
        pending.push_back(neighbour);
      }
    }
  }
  safe_cells_.swap(connected);
}

std::vector<SpawnPose> SafeSpawnGrid::sample(
  std::size_t count, std::uint64_t seed) const
{
  if (count == 0U) {return {};}
  if (seed == 0U) {throw std::invalid_argument("sample seed must be non-zero");}
  std::vector<std::size_t> candidates;
  candidates.reserve(safeCellCount());
  for (std::size_t cell = 0U; cell < safe_cells_.size(); ++cell) {
    if (safe_cells_[cell] != 0U) {candidates.push_back(cell);}
  }
  std::mt19937_64 engine(seed);
  std::shuffle(candidates.begin(), candidates.end(), engine);
  std::uniform_real_distribution<double> yaw_distribution(-kPi, kPi);
  std::vector<SpawnPose> result;
  result.reserve(count);
  for (const auto cell : candidates) {
    const auto point = cellCenter(cell);
    const bool separated = std::all_of(
      result.begin(), result.end(), [&point, this](const auto & pose) {
        return std::hypot(point.x - pose.x, point.y - pose.y) >=
               parameters_.minimum_spawn_separation;
      });
    if (!separated) {continue;}
    result.push_back({point.x, point.y, parameters_.spawn_z, yaw_distribution(engine)});
    if (result.size() == count) {break;}
  }
  if (result.size() != count) {
    throw std::runtime_error("not enough separated safe spawn cells in the reachable region");
  }
  return result;
}

bool SafeSpawnGrid::isSafe(double x, double y) const
{
  if (!std::isfinite(x) || !std::isfinite(y)) {return false;}
  const auto grid_x = static_cast<std::int64_t>(std::floor(
      (x - geometry_.sampling_bounds.minimum_x) / parameters_.resolution));
  const auto grid_y = static_cast<std::int64_t>(std::floor(
      (y - geometry_.sampling_bounds.minimum_y) / parameters_.resolution));
  if (grid_x < 0 || grid_y < 0 || grid_x >= static_cast<std::int64_t>(width_) ||
    grid_y >= static_cast<std::int64_t>(height_))
  {
    return false;
  }
  return safe_cells_[index(
    static_cast<std::size_t>(grid_x), static_cast<std::size_t>(grid_y))] != 0U;
}

std::size_t SafeSpawnGrid::safeCellCount() const
{
  return static_cast<std::size_t>(std::count(
      safe_cells_.begin(), safe_cells_.end(), static_cast<std::uint8_t>(1U)));
}

const Bounds2D & SafeSpawnGrid::bounds() const {return geometry_.sampling_bounds;}
double SafeSpawnGrid::resolution() const {return parameters_.resolution;}

void SafeSpawnGrid::writeDebugPgm(
  const std::string & path, const std::vector<SpawnPose> & samples) const
{
  std::vector<std::uint8_t> pixels(safe_cells_.size(), 80U);
  for (std::size_t index = 0U; index < pixels.size(); ++index) {
    if (safe_cells_[index] != 0U) {pixels[index] = 240U;}
  }
  for (const auto & sample : samples) {
    const auto x = static_cast<std::int64_t>(std::floor(
        (sample.x - geometry_.sampling_bounds.minimum_x) / parameters_.resolution));
    const auto y = static_cast<std::int64_t>(std::floor(
        (sample.y - geometry_.sampling_bounds.minimum_y) / parameters_.resolution));
    if (x >= 0 && y >= 0 && x < static_cast<std::int64_t>(width_) &&
      y < static_cast<std::int64_t>(height_))
    {
      pixels[index(static_cast<std::size_t>(x), static_cast<std::size_t>(y))] = 0U;
    }
  }
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) {throw std::runtime_error("failed to open debug PGM: " + path);}
  output << "P5\n" << width_ << ' ' << height_ << "\n255\n";
  for (std::size_t y = height_; y-- > 0U; ) {
    output.write(
      reinterpret_cast<const char *>(pixels.data() + y * width_),
      static_cast<std::streamsize>(width_));
  }
  if (!output) {throw std::runtime_error("failed to write debug PGM: " + path);}
}

void SafeSpawnGrid::resolveSupportLayer()
{
  // Which candidates lie under the reference point decides what "the floor"
  // means for this world. Shape alone cannot: a ground slab, a doorway header
  // and a canopy are all thin, flat and horizontal, and only their height
  // relative to where the robot starts tells them apart.
  const Point2D reference{parameters_.reference_x, parameters_.reference_y};
  std::vector<double> heights_under_reference;
  for (const auto & candidate : geometry_.support_candidates) {
    if (pointInSupport(reference, candidate.surface)) {
      heights_under_reference.push_back(candidate.surface.height);
    }
  }
  if (heights_under_reference.empty()) {
    throw std::runtime_error(
            "the reference point is not on any support surface, so there is no "
            "floor to sample from");
  }
  const auto extremes = std::minmax_element(
    heights_under_reference.begin(), heights_under_reference.end());
  if (*extremes.second - *extremes.first > parameters_.traversable_step) {
    // Choosing the highest silently would make the ground beneath it an
    // obstacle and leave almost nothing safe, which reads as a broken world
    // rather than an unsupported one.
    throw std::runtime_error(
            "multi-level support at the reference point is not supported: found "
            "surfaces at " + formatHeight(*extremes.first) + " m and " +
            formatHeight(*extremes.second) + " m");
  }
  reference_height_ = *extremes.first;

  for (auto & candidate : geometry_.support_candidates) {
    if (std::abs(candidate.surface.height - reference_height_) <=
      parameters_.traversable_step)
    {
      supports_.push_back(candidate.surface);
    } else {
      // Not the floor, so it is something the robot has to clear. The vertical
      // test decides whether it actually blocks: a header at 0.70 m is driven
      // under, a 0.08 m kerb is not.
      ++demoted_supports_;
      geometry_.obstacles.push_back(candidate.body);
    }
  }
}

Bounds2D SafeSpawnGrid::clipSamplingBounds() const
{
  // The floor decides how far the world can extend, but a ground plane is
  // usually far larger than the building on it: slam_world's is 100 x 100 m
  // around a 12 x 10 m interior, which spent 4,000,000 cells -- exactly the
  // cap -- to describe 25,000 useful ones. The walls bound the reachable
  // region anyway, so the obstacles say where it is worth looking.
  //
  // Only the grid's extent is clipped. The obstacle list stays whole: a wall
  // outside these bounds must still block the cells inside them, and filtering
  // the obstacles to match would leave the boundary cells looking free.
  Bounds2D support_bounds{};
  bool have_support = false;
  for (const auto & support : supports_) {
    merge(support_bounds, surfaceBounds(support), have_support);
  }
  if (!have_support) {return geometry_.sampling_bounds;}

  Bounds2D obstacle_extent{};
  bool have_obstacle = false;
  for (const auto & obstacle : geometry_.obstacles) {
    merge(obstacle_extent, obstacleBounds(obstacle), have_obstacle);
  }
  if (!have_obstacle) {return snapToLattice(support_bounds);}

  // Enough room that an obstacle sitting on the clipped edge is still fully
  // inflated inside it, plus a cell so rounding cannot eat into that.
  const double room = parameters_.robot_circumscribed_radius +
    parameters_.safety_margin + parameters_.non_static_extra_margin +
    parameters_.resolution;
  Bounds2D clipped{
    std::max(support_bounds.minimum_x, obstacle_extent.minimum_x - room),
    std::max(support_bounds.minimum_y, obstacle_extent.minimum_y - room),
    std::min(support_bounds.maximum_x, obstacle_extent.maximum_x + room),
    std::min(support_bounds.maximum_y, obstacle_extent.maximum_y + room)};
  // The reference point has to survive the clip, or the grid is built around a
  // start position it does not contain.
  clipped.minimum_x = std::min(clipped.minimum_x, parameters_.reference_x - room);
  clipped.minimum_y = std::min(clipped.minimum_y, parameters_.reference_y - room);
  clipped.maximum_x = std::max(clipped.maximum_x, parameters_.reference_x + room);
  clipped.maximum_y = std::max(clipped.maximum_y, parameters_.reference_y + room);
  if (clipped.minimum_x >= clipped.maximum_x || clipped.minimum_y >= clipped.maximum_y) {
    return snapToLattice(support_bounds);
  }
  return snapToLattice(clipped);
}

Bounds2D SafeSpawnGrid::snapToLattice(const Bounds2D & bounds) const
{
  // Anchored at the world origin, so a wall moved by less than a cell does not
  // shift every cell centre in the world and change every sampled pose. Both
  // ends move outward, so snapping never eats into the room reserved above.
  const double resolution = parameters_.resolution;
  return {
    std::floor(bounds.minimum_x / resolution) * resolution,
    std::floor(bounds.minimum_y / resolution) * resolution,
    std::ceil(bounds.maximum_x / resolution) * resolution,
    std::ceil(bounds.maximum_y / resolution) * resolution};
}

double SafeSpawnGrid::referenceHeight() const {return reference_height_;}
std::size_t SafeSpawnGrid::demotedSupportCount() const {return demoted_supports_;}

std::size_t SafeSpawnGrid::index(std::size_t x, std::size_t y) const
{
  return y * width_ + x;
}

Point2D SafeSpawnGrid::cellCenter(std::size_t cell) const
{
  const auto x = cell % width_;
  const auto y = cell / width_;
  return {
    geometry_.sampling_bounds.minimum_x + (static_cast<double>(x) + 0.5) *
    parameters_.resolution,
    geometry_.sampling_bounds.minimum_y + (static_cast<double>(y) + 0.5) *
    parameters_.resolution};
}

std::uint64_t makeSpawnSeed()
{
  std::random_device device;
  const auto time = static_cast<std::uint64_t>(
    std::chrono::steady_clock::now().time_since_epoch().count());
  const std::uint64_t seed =
    (static_cast<std::uint64_t>(device()) << 32U) ^ device() ^ time;
  return seed == 0U ? 1U : seed;
}

}  // namespace slam_robot_gazebo
