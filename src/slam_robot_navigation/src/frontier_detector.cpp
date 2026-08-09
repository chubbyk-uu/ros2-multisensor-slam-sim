// Copyright 2026 Jerry

#include "slam_robot_navigation/frontier_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

namespace slam_robot_navigation
{
namespace
{

constexpr int kNeighbors8[8][2] = {
  {-1, -1}, {0, -1}, {1, -1}, {-1, 0},
  {1, 0}, {-1, 1}, {0, 1}, {1, 1}};
constexpr int kNeighbors4[4][2] = {{0, -1}, {-1, 0}, {1, 0}, {0, 1}};

bool validCell(int x, int y, std::size_t width, std::size_t height)
{
  return x >= 0 && y >= 0 && static_cast<std::size_t>(x) < width &&
         static_cast<std::size_t>(y) < height;
}

std::size_t indexOf(std::size_t x, std::size_t y, std::size_t width)
{
  return y * width + x;
}

}  // namespace

FrontierDetector::FrontierDetector(FrontierDetectorParameters parameters)
: parameters_(std::move(parameters))
{
  if (parameters_.free_maximum < 0 ||
    parameters_.occupied_minimum <= parameters_.free_maximum ||
    parameters_.minimum_cluster_cells == 0 ||
    !std::isfinite(parameters_.minimum_clearance) || parameters_.minimum_clearance < 0.0 ||
    !std::isfinite(parameters_.information_gain_weight) ||
    !std::isfinite(parameters_.distance_weight) ||
    !std::isfinite(parameters_.clearance_weight))
  {
    throw std::invalid_argument("invalid frontier detector parameters");
  }
}

std::vector<FrontierCandidate> FrontierDetector::detect(
  const nav_msgs::msg::OccupancyGrid & map, double robot_x, double robot_y) const
{
  const std::size_t width = map.info.width;
  const std::size_t height = map.info.height;
  const double resolution = map.info.resolution;
  if (width == 0 || height == 0 || map.data.size() != width * height ||
    !std::isfinite(resolution) || resolution <= 0.0 ||
    !std::isfinite(robot_x) || !std::isfinite(robot_y))
  {
    throw std::invalid_argument("invalid occupancy grid or robot pose");
  }

  const auto is_free = [this, &map](std::size_t index) {
      return map.data[index] >= 0 && map.data[index] <= parameters_.free_maximum;
    };
  const auto is_unknown = [&map](std::size_t index) {return map.data[index] < 0;};

  std::vector<double> clearance(width * height, std::numeric_limits<double>::infinity());
  std::queue<std::size_t> distance_queue;
  for (std::size_t i = 0; i < map.data.size(); ++i) {
    if (map.data[i] >= parameters_.occupied_minimum) {
      clearance[i] = 0.0;
      distance_queue.push(i);
    }
  }
  while (!distance_queue.empty()) {
    const std::size_t current = distance_queue.front();
    distance_queue.pop();
    const int x = static_cast<int>(current % width);
    const int y = static_cast<int>(current / width);
    for (const auto & neighbor : kNeighbors4) {
      const int nx = x + neighbor[0];
      const int ny = y + neighbor[1];
      if (!validCell(nx, ny, width, height)) {continue;}
      const std::size_t next = indexOf(nx, ny, width);
      const double proposed = clearance[current] + resolution;
      if (proposed < clearance[next]) {
        clearance[next] = proposed;
        distance_queue.push(next);
      }
    }
  }

  std::vector<bool> frontier(width * height, false);
  for (std::size_t y = 0; y < height; ++y) {
    for (std::size_t x = 0; x < width; ++x) {
      const std::size_t cell = indexOf(x, y, width);
      if (!is_free(cell) || clearance[cell] < parameters_.minimum_clearance) {continue;}
      for (const auto & neighbor : kNeighbors4) {
        const int nx = static_cast<int>(x) + neighbor[0];
        const int ny = static_cast<int>(y) + neighbor[1];
        if (validCell(nx, ny, width, height) && is_unknown(indexOf(nx, ny, width))) {
          frontier[cell] = true;
          break;
        }
      }
    }
  }

  std::vector<bool> visited(width * height, false);
  std::vector<FrontierCandidate> candidates;
  for (std::size_t seed = 0; seed < frontier.size(); ++seed) {
    if (!frontier[seed] || visited[seed]) {continue;}
    std::queue<std::size_t> cluster_queue;
    std::vector<std::size_t> cluster;
    visited[seed] = true;
    cluster_queue.push(seed);
    while (!cluster_queue.empty()) {
      const std::size_t current = cluster_queue.front();
      cluster_queue.pop();
      cluster.push_back(current);
      const int x = static_cast<int>(current % width);
      const int y = static_cast<int>(current / width);
      for (const auto & neighbor : kNeighbors8) {
        const int nx = x + neighbor[0];
        const int ny = y + neighbor[1];
        if (!validCell(nx, ny, width, height)) {continue;}
        const std::size_t next = indexOf(nx, ny, width);
        if (frontier[next] && !visited[next]) {
          visited[next] = true;
          cluster_queue.push(next);
        }
      }
    }
    if (cluster.size() < parameters_.minimum_cluster_cells) {continue;}

    double centroid_x = 0.0;
    double centroid_y = 0.0;
    for (const std::size_t cell : cluster) {
      centroid_x += static_cast<double>(cell % width);
      centroid_y += static_cast<double>(cell / width);
    }
    centroid_x /= static_cast<double>(cluster.size());
    centroid_y /= static_cast<double>(cluster.size());

    const auto goal = *std::min_element(
      cluster.begin(), cluster.end(),
      [&](std::size_t lhs, std::size_t rhs) {
        const double lhs_dx = static_cast<double>(lhs % width) - centroid_x;
        const double lhs_dy = static_cast<double>(lhs / width) - centroid_y;
        const double rhs_dx = static_cast<double>(rhs % width) - centroid_x;
        const double rhs_dy = static_cast<double>(rhs / width) - centroid_y;
        const double lhs_cost = lhs_dx * lhs_dx + lhs_dy * lhs_dy - clearance[lhs];
        const double rhs_cost = rhs_dx * rhs_dx + rhs_dy * rhs_dy - clearance[rhs];
        return lhs_cost < rhs_cost;
      });
    FrontierCandidate candidate;
    candidate.cell_x = goal % width;
    candidate.cell_y = goal / width;
    candidate.x = map.info.origin.position.x +
      (static_cast<double>(candidate.cell_x) + 0.5) * resolution;
    candidate.y = map.info.origin.position.y +
      (static_cast<double>(candidate.cell_y) + 0.5) * resolution;
    candidate.cluster_cells = cluster.size();
    candidate.information_gain = static_cast<double>(cluster.size()) * resolution;
    candidate.clearance = std::min(
      clearance[goal], std::hypot(static_cast<double>(width), static_cast<double>(height)) *
      resolution);
    double unknown_direction_x = 0.0;
    double unknown_direction_y = 0.0;
    for (const auto & neighbor : kNeighbors8) {
      const int nx = static_cast<int>(candidate.cell_x) + neighbor[0];
      const int ny = static_cast<int>(candidate.cell_y) + neighbor[1];
      if (validCell(nx, ny, width, height) && is_unknown(indexOf(nx, ny, width))) {
        unknown_direction_x += static_cast<double>(neighbor[0]);
        unknown_direction_y += static_cast<double>(neighbor[1]);
      }
    }
    candidate.yaw = std::atan2(unknown_direction_y, unknown_direction_x);
    candidate.robot_distance = std::hypot(candidate.x - robot_x, candidate.y - robot_y);
    candidate.score = parameters_.information_gain_weight * candidate.information_gain -
      parameters_.distance_weight * candidate.robot_distance +
      parameters_.clearance_weight * candidate.clearance;
    candidates.push_back(candidate);
  }
  std::sort(candidates.begin(), candidates.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.score > rhs.score;
    });
  return candidates;
}

}  // namespace slam_robot_navigation
