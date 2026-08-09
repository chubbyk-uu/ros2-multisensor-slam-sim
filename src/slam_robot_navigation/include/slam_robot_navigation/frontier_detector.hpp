// Copyright 2026 Jerry

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace slam_robot_navigation
{

struct FrontierDetectorParameters
{
  std::int8_t free_maximum{20};
  std::int8_t occupied_minimum{65};
  std::size_t minimum_cluster_cells{8};
  double minimum_clearance{0.25};
  double information_gain_weight{2.0};
  double distance_weight{1.0};
  double clearance_weight{0.5};
};

struct FrontierCandidate
{
  std::size_t cell_x{0};
  std::size_t cell_y{0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double information_gain{0.0};
  double clearance{0.0};
  double robot_distance{0.0};
  double score{0.0};
  std::size_t cluster_cells{0};
};

class FrontierDetector
{
public:
  explicit FrontierDetector(FrontierDetectorParameters parameters);

  std::vector<FrontierCandidate> detect(
    const nav_msgs::msg::OccupancyGrid & map, double robot_x, double robot_y) const;

private:
  FrontierDetectorParameters parameters_;
};

}  // namespace slam_robot_navigation
