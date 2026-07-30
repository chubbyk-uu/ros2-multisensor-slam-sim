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
    !std::isfinite(parameters_.hit_probability) ||
    !std::isfinite(parameters_.miss_probability) ||
    !std::isfinite(parameters_.minimum_probability) ||
    !std::isfinite(parameters_.maximum_probability) ||
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
  const double grid_x =
    std::floor(static_cast<double>(point.x) / parameters_.resolution);
  const double grid_y =
    std::floor(static_cast<double>(point.y) / parameters_.resolution);
  if (grid_x < std::numeric_limits<int>::min() ||
    grid_x > std::numeric_limits<int>::max() ||
    grid_y < std::numeric_limits<int>::min() ||
    grid_y > std::numeric_limits<int>::max())
  {
    throw std::overflow_error(
            "Occupancy grid coordinate exceeds integer range");
  }
  return GridIndex{
    static_cast<int>(grid_x),
    static_cast<int>(grid_y)};
}

OccupancyGridMap::GridIndex OccupancyGridMap::blockIndex(
  const GridIndex & cell)
{
  const auto floor_divide = [](const int value) {
      int quotient = value / kBlockSize;
      if (value % kBlockSize < 0) {
        --quotient;
      }
      return quotient;
    };
  return GridIndex{floor_divide(cell.x), floor_divide(cell.y)};
}

std::size_t OccupancyGridMap::localCellOffset(
  const GridIndex & cell,
  const GridIndex & block)
{
  const int local_x = cell.x - block.x * kBlockSize;
  const int local_y = cell.y - block.y * kBlockSize;
  return static_cast<std::size_t>(local_y * kBlockSize + local_x);
}

void OccupancyGridMap::updateCell(
  const GridIndex & index,
  const double log_odds_increment)
{
  const GridIndex block_index = blockIndex(index);
  auto [block_iterator, unused] = blocks_.try_emplace(block_index);
  (void)unused;
  CellBlock & block = block_iterator->second;
  const std::size_t offset = localCellOffset(index, block_index);
  const bool inserted = !block.observed.test(offset);
  if (inserted) {
    block.observed.set(offset);
    ++observed_cell_count_;
  }
  block.log_odds[offset] = static_cast<float>(std::clamp(
      static_cast<double>(block.log_odds[offset]) + log_odds_increment,
      minimum_log_odds_,
      maximum_log_odds_));

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
  blocks_.clear();
  observed_cell_count_ = 0U;
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

  const int64_t origin_x =
    static_cast<int64_t>(minimum_x_) - parameters_.padding_cells;
  const int64_t origin_y =
    static_cast<int64_t>(minimum_y_) - parameters_.padding_cells;
  const int64_t padded_maximum_x =
    static_cast<int64_t>(maximum_x_) + parameters_.padding_cells;
  const int64_t padded_maximum_y =
    static_cast<int64_t>(maximum_y_) + parameters_.padding_cells;
  if (origin_x < std::numeric_limits<int>::min() ||
    origin_x > std::numeric_limits<int>::max() ||
    origin_y < std::numeric_limits<int>::min() ||
    origin_y > std::numeric_limits<int>::max())
  {
    throw std::overflow_error("Occupancy grid origin exceeds integer range");
  }
  result.origin_cell_x = static_cast<int>(origin_x);
  result.origin_cell_y = static_cast<int>(origin_y);
  result.width = static_cast<std::size_t>(
    padded_maximum_x - origin_x + 1);
  result.height = static_cast<std::size_t>(
    padded_maximum_y - origin_y + 1);

  if (result.width >
    std::numeric_limits<std::size_t>::max() / result.height)
  {
    throw std::overflow_error("Occupancy grid dimensions overflow");
  }
  result.data.assign(result.width * result.height, int8_t{-1});

  for (const auto & [block_index, block] : blocks_) {
    for (int local_y = 0; local_y < kBlockSize; ++local_y) {
      for (int local_x = 0; local_x < kBlockSize; ++local_x) {
        const auto offset = static_cast<std::size_t>(
          local_y * kBlockSize + local_x);
        if (!block.observed.test(offset)) {
          continue;
        }
        const int cell_x = block_index.x * kBlockSize + local_x;
        const int cell_y = block_index.y * kBlockSize + local_y;
        const auto column =
          static_cast<std::size_t>(cell_x - result.origin_cell_x);
        const auto row =
          static_cast<std::size_t>(cell_y - result.origin_cell_y);
        const double probability =
          1.0 / (1.0 + std::exp(-block.log_odds[offset]));
        result.data[row * result.width + column] = static_cast<int8_t>(
          std::clamp(std::lround(probability * 100.0), 0L, 100L));
      }
    }
  }
  return result;
}

std::size_t OccupancyGridMap::observedCellCount() const
{
  return observed_cell_count_;
}

std::size_t OccupancyGridMap::allocatedBlockCount() const
{
  return blocks_.size();
}

}  // namespace slam_robot_slam
