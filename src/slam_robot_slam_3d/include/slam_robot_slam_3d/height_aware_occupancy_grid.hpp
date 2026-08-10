#ifndef SLAM_ROBOT_SLAM_3D__HEIGHT_AWARE_OCCUPANCY_GRID_HPP_
#define SLAM_ROBOT_SLAM_3D__HEIGHT_AWARE_OCCUPANCY_GRID_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Geometry>

#include "slam_robot_slam/occupancy_grid_map.hpp"
#include "slam_robot_slam_3d/global_keyframe_map.hpp"

namespace slam_robot_slam_3d
{

struct HeightAwareOccupancyGridParameters
{
  slam_robot_slam::OccupancyGridMapParameters grid;
  double minimum_obstacle_height{0.05};
  double maximum_obstacle_height{0.45};
  std::size_t keyframes_per_batch{4U};
  std::int8_t free_maximum{20};
  std::int8_t occupied_minimum{65};
};

// Projects global keyframes into a probabilistic 2D grid. Only returns inside
// the obstacle height band mark cells occupied; returns below it contribute
// ray-traced free space, and returns above it are ignored. navigationSnapshot
// converts that internal probability map to Nav2's unambiguous -1/0/100
// contract: cells without enough free or occupied evidence remain unknown.
// Mapping appends new keyframes in place, while a pose-graph correction
// replays a consistent full snapshot through begin() in bounded batches.
class HeightAwareOccupancyGrid
{
public:
  explicit HeightAwareOccupancyGrid(HeightAwareOccupancyGridParameters parameters);
  void begin(std::vector<GlobalKeyframe> keyframes, std::vector<Eigen::Isometry3d> poses);
  // Adds a keyframe to an already-built grid.  Global pose corrections still
  // require begin(), which deliberately replays a consistent full snapshot.
  void append(const GlobalKeyframe & keyframe, const Eigen::Isometry3d & pose);
  bool processBatch();
  bool active() const;
  std::size_t processedKeyframes() const;
  std::size_t totalKeyframes() const;
  // Full log-odds projection for diagnosis only. It is not a Nav2 input.
  slam_robot_slam::OccupancyGridSnapshot snapshot() const;
  slam_robot_slam::OccupancyGridSnapshot navigationSnapshot() const;

private:
  void integrate(const GlobalKeyframe & keyframe, const Eigen::Isometry3d & pose);

  HeightAwareOccupancyGridParameters parameters_;
  slam_robot_slam::OccupancyGridMap grid_;
  std::vector<GlobalKeyframe> keyframes_;
  std::vector<Eigen::Isometry3d> poses_;
  std::size_t next_{0U};
};

}  // namespace slam_robot_slam_3d

#endif  // SLAM_ROBOT_SLAM_3D__HEIGHT_AWARE_OCCUPANCY_GRID_HPP_
