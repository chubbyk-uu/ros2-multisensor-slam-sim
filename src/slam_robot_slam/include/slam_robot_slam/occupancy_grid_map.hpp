#ifndef SLAM_ROBOT_SLAM__OCCUPANCY_GRID_MAP_HPP_
#define SLAM_ROBOT_SLAM__OCCUPANCY_GRID_MAP_HPP_

#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "slam_robot_slam/point2d.hpp"

namespace slam_robot_slam
{

struct OccupancyGridMapParameters
{
  double resolution{0.05};
  double hit_probability{0.70};
  double miss_probability{0.40};
  double minimum_probability{0.12};
  double maximum_probability{0.97};
  int padding_cells{2};
};

struct OccupancyGridSnapshot
{
  double resolution{0.05};
  int origin_cell_x{0};
  int origin_cell_y{0};
  std::size_t width{0U};
  std::size_t height{0U};
  std::vector<int8_t> data;
};

class OccupancyGridMap
{
public:
  explicit OccupancyGridMap(
    const OccupancyGridMapParameters & parameters);

  void updateRay(
    const Point2D & origin,
    const Point2D & endpoint,
    bool endpoint_is_hit);

  void clear();

  OccupancyGridSnapshot snapshot() const;

  std::size_t observedCellCount() const;
  std::size_t allocatedBlockCount() const;

private:
  static constexpr int kBlockSize = 16;
  static constexpr std::size_t kCellsPerBlock =
    static_cast<std::size_t>(kBlockSize * kBlockSize);

  struct GridIndex
  {
    int x;
    int y;

    bool operator==(const GridIndex & other) const
    {
      return x == other.x && y == other.y;
    }
  };

  struct CellBlock
  {
    std::array<float, kCellsPerBlock> log_odds{};
    std::bitset<kCellsPerBlock> observed;
  };

  struct GridIndexHash
  {
    std::size_t operator()(const GridIndex & index) const;
  };

  GridIndex worldToGrid(const Point2D & point) const;
  static GridIndex blockIndex(const GridIndex & cell);
  static std::size_t localCellOffset(
    const GridIndex & cell,
    const GridIndex & block);
  void updateCell(const GridIndex & index, double log_odds_increment);

  OccupancyGridMapParameters parameters_;
  double hit_log_odds_;
  double miss_log_odds_;
  double minimum_log_odds_;
  double maximum_log_odds_;
  std::unordered_map<GridIndex, CellBlock, GridIndexHash> blocks_;
  std::size_t observed_cell_count_{0U};
  bool has_bounds_{false};
  int minimum_x_{0};
  int maximum_x_{0};
  int minimum_y_{0};
  int maximum_y_{0};
};

}  // namespace slam_robot_slam

#endif  // SLAM_ROBOT_SLAM__OCCUPANCY_GRID_MAP_HPP_
