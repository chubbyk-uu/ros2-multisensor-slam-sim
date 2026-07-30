#include "slam_robot_slam/occupancy_grid_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace slam_robot_slam
{
namespace
{

double probabilityToLogOdds(const double probability)
{
  return std::log(probability / (1.0 - probability));
}

}  // namespace

OccupancyGridMap::OccupancyGridMap(
  const OccupancyGridMapParameters & parameters)
: parameters_(parameters),
  hit_log_odds_(probabilityToLogOdds(parameters.hit_probability)),
  miss_log_odds_(probabilityToLogOdds(parameters.miss_probability)),
  minimum_log_odds_(probabilityToLogOdds(parameters.minimum_probability)),
  maximum_log_odds_(probabilityToLogOdds(parameters.maximum_probability))
{
  if (!std::isfinite(parameters_.resolution) ||
    parameters_.resolution <= 0.0 ||
    parameters_.hit_probability <= 0.5 ||
    parameters_.hit_probability >= 1.0 ||
    parameters_.miss_probability <= 0.0 ||
    parameters_.miss_probability >= 0.5 ||
    parameters_.minimum_probability <= 0.0 ||
    parameters_.minimum_probability >= 0.5 ||
    parameters_.maximum_probability <= 0.5 ||
    parameters_.maximum_probability >= 1.0 ||
    parameters_.minimum_probability >= parameters_.maximum_probability ||
    parameters_.padding_cells < 0)
  {
    throw std::invalid_argument("Invalid occupancy grid map parameters");
  }
}

std::size_t OccupancyGridMap::GridIndexHash::operator()(
  const GridIndex & index) const
{
  const auto x = static_cast<uint32_t>(index.x);
  const auto y = static_cast<uint32_t>(index.y);
  return static_cast<std::size_t>(
    (static_cast<uint64_t>(x) << 32U) | static_cast<uint64_t>(y));
}

OccupancyGridMap::GridIndex OccupancyGridMap::worldToGrid(
  const Point2D & point) const
{
  return GridIndex{
    static_cast<int>(
      std::floor(static_cast<double>(point.x) / parameters_.resolution)),
    static_cast<int>(
      std::floor(static_cast<double>(point.y) / parameters_.resolution))};
}

void OccupancyGridMap::updateCell(
  const GridIndex & index,
  const double log_odds_increment)
{
  auto [iterator, inserted] = cells_.try_emplace(index, 0.0);
  iterator->second = std::clamp(
    iterator->second + log_odds_increment,
    minimum_log_odds_,
    maximum_log_odds_);

  if (!has_bounds_) {
    minimum_x_ = maximum_x_ = index.x;
    minimum_y_ = maximum_y_ = index.y;
    has_bounds_ = true;
    return;
  }
  if (inserted) {
    minimum_x_ = std::min(minimum_x_, index.x);
    maximum_x_ = std::max(maximum_x_, index.x);
    minimum_y_ = std::min(minimum_y_, index.y);
    maximum_y_ = std::max(maximum_y_, index.y);
  }
}

void OccupancyGridMap::updateRay(
  const Point2D & origin,
  const Point2D & endpoint,
  const bool endpoint_is_hit)
{
  if (!std::isfinite(origin.x) || !std::isfinite(origin.y) ||
    !std::isfinite(endpoint.x) || !std::isfinite(endpoint.y))
  {
    return;
  }

  GridIndex current = worldToGrid(origin);
  const GridIndex end = worldToGrid(endpoint);
  const int delta_x = std::abs(end.x - current.x);
  const int delta_y = std::abs(end.y - current.y);
  const int step_x = current.x < end.x ? 1 : -1;
  const int step_y = current.y < end.y ? 1 : -1;
  int error = delta_x - delta_y;

  while (!(current == end)) {
    updateCell(current, miss_log_odds_);
    const int doubled_error = 2 * error;
    if (doubled_error > -delta_y) {
      error -= delta_y;
      current.x += step_x;
    }
    if (doubled_error < delta_x) {
      error += delta_x;
      current.y += step_y;
    }
  }

  updateCell(end, endpoint_is_hit ? hit_log_odds_ : miss_log_odds_);
}

void OccupancyGridMap::clear()
{
  cells_.clear();
  has_bounds_ = false;
  minimum_x_ = 0;
  maximum_x_ = 0;
  minimum_y_ = 0;
  maximum_y_ = 0;
}

OccupancyGridSnapshot OccupancyGridMap::snapshot() const
{
  OccupancyGridSnapshot result;
  result.resolution = parameters_.resolution;
  if (!has_bounds_) {
    return result;
  }

  result.origin_cell_x = minimum_x_ - parameters_.padding_cells;
  result.origin_cell_y = minimum_y_ - parameters_.padding_cells;
  const int padded_maximum_x = maximum_x_ + parameters_.padding_cells;
  const int padded_maximum_y = maximum_y_ + parameters_.padding_cells;
  result.width = static_cast<std::size_t>(
    padded_maximum_x - result.origin_cell_x + 1);
  result.height = static_cast<std::size_t>(
    padded_maximum_y - result.origin_cell_y + 1);

  if (result.width >
    std::numeric_limits<std::size_t>::max() / result.height)
  {
    throw std::overflow_error("Occupancy grid dimensions overflow");
  }
  result.data.assign(result.width * result.height, int8_t{-1});

  for (const auto & [index, log_odds] : cells_) {
    const auto column =
      static_cast<std::size_t>(index.x - result.origin_cell_x);
    const auto row =
      static_cast<std::size_t>(index.y - result.origin_cell_y);
    const double probability = 1.0 / (1.0 + std::exp(-log_odds));
    result.data[row * result.width + column] = static_cast<int8_t>(
      std::clamp(std::lround(probability * 100.0), 0L, 100L));
  }
  return result;
}

std::size_t OccupancyGridMap::observedCellCount() const
{
  return cells_.size();
}

}  // namespace slam_robot_slam
