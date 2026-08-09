#include "slam_robot_slam_3d/height_aware_occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace slam_robot_slam_3d
{

HeightAwareOccupancyGrid::HeightAwareOccupancyGrid(
  HeightAwareOccupancyGridParameters parameters)
: parameters_(std::move(parameters)), grid_(parameters_.grid)
{
  if (!std::isfinite(parameters_.minimum_obstacle_height) ||
    !std::isfinite(parameters_.maximum_obstacle_height) ||
    parameters_.minimum_obstacle_height < 0.0 ||
    parameters_.minimum_obstacle_height >= parameters_.maximum_obstacle_height ||
    parameters_.keyframes_per_batch == 0U)
  {
    throw std::invalid_argument("height-aware occupancy grid parameters are invalid");
  }
}

void HeightAwareOccupancyGrid::begin(
  std::vector<GlobalKeyframe> keyframes, std::vector<Eigen::Isometry3d> poses)
{
  if (keyframes.empty() || keyframes.size() != poses.size()) {
    throw std::invalid_argument("occupancy rebuild snapshot is inconsistent");
  }
  grid_.clear();
  keyframes_ = std::move(keyframes);
  poses_ = std::move(poses);
  next_ = 0U;
}

void HeightAwareOccupancyGrid::append(
  const GlobalKeyframe & keyframe, const Eigen::Isometry3d & pose)
{
  if (active()) {
    throw std::logic_error("cannot append while an occupancy rebuild is active");
  }
  integrate(keyframe, pose);
}

bool HeightAwareOccupancyGrid::processBatch()
{
  if (!active()) {
    return false;
  }
  const auto end = std::min(keyframes_.size(), next_ + parameters_.keyframes_per_batch);
  for (; next_ < end; ++next_) {
    integrate(keyframes_[next_], poses_[next_]);
  }
  return !active();
}

bool HeightAwareOccupancyGrid::active() const
{
  return next_ < keyframes_.size();
}

std::size_t HeightAwareOccupancyGrid::processedKeyframes() const
{
  return next_;
}

std::size_t HeightAwareOccupancyGrid::totalKeyframes() const
{
  return keyframes_.size();
}

slam_robot_slam::OccupancyGridSnapshot HeightAwareOccupancyGrid::snapshot() const
{
  return grid_.snapshot();
}

void HeightAwareOccupancyGrid::integrate(
  const GlobalKeyframe & keyframe, const Eigen::Isometry3d & pose)
{
  if (!keyframe.filtered_scan || !pose.matrix().allFinite()) {
    throw std::invalid_argument("occupancy keyframe is invalid");
  }
  const Eigen::Isometry3d map_from_sensor = pose * keyframe.base_to_sensor;
  const Eigen::Vector3d origin = map_from_sensor.translation();
  for (const auto & point : *keyframe.filtered_scan) {
    const Eigen::Vector3d endpoint = map_from_sensor * Eigen::Vector3d(point.x, point.y, point.z);
    if (!endpoint.allFinite()) {
      continue;
    }
    const slam_robot_slam::Point2D origin_xy{
      static_cast<float>(origin.x()), static_cast<float>(origin.y())};
    const slam_robot_slam::Point2D endpoint_xy{
      static_cast<float>(endpoint.x()), static_cast<float>(endpoint.y())};
    if (endpoint.z() < parameters_.minimum_obstacle_height) {
      // A return below the navigation obstacle band is ground evidence.  It
      // cannot occupy a 2D navigation cell, but its observed line of sight
      // establishes free space.  Returns above the band remain excluded:
      // projecting a ceiling or overhang ray into 2D would incorrectly clear
      // an obstacle below it.
      grid_.updateRay(origin_xy, endpoint_xy, false);
      continue;
    }
    if (endpoint.z() > parameters_.maximum_obstacle_height) {
      continue;
    }
    grid_.updateRay(origin_xy, endpoint_xy, true);
  }
}

}  // namespace slam_robot_slam_3d
