#include "slam_robot_slam_3d/global_keyframe_map.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <utility>

#include <pcl/common/point_tests.h>

namespace slam_robot_slam_3d
{

std::size_t GlobalKeyframeMap::add(GlobalKeyframe keyframe)
{
  validateKeyframe(keyframe);
  std::unique_lock<std::shared_mutex> lock(mutex_);
  keyframe.id = keyframes_.size();
  point_count_ += keyframe.filtered_scan->size();
  keyframes_.push_back(std::move(keyframe));
  return keyframes_.back().id;
}

std::vector<GlobalKeyframe> GlobalKeyframeMap::snapshot() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return keyframes_;
}

void GlobalKeyframeMap::replace(std::vector<GlobalKeyframe> keyframes)
{
  std::size_t points = 0U;
  for (std::size_t index = 0U; index < keyframes.size(); ++index) {
    if (keyframes[index].id != index) {
      throw std::invalid_argument("loaded global keyframes must use contiguous ids");
    }
    validateKeyframe(keyframes[index]);
    points += keyframes[index].filtered_scan->size();
  }
  std::unique_lock<std::shared_mutex> lock(mutex_);
  keyframes_ = std::move(keyframes);
  point_count_ = points;
}

std::size_t GlobalKeyframeMap::size() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return keyframes_.size();
}

std::size_t GlobalKeyframeMap::pointCount() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return point_count_;
}

void GlobalKeyframeMap::validateKeyframe(const GlobalKeyframe & keyframe)
{
  if (!keyframe.filtered_scan || keyframe.filtered_scan->empty()) {
    throw std::invalid_argument("global keyframe scan must not be empty");
  }
  if (!std::all_of(
      keyframe.filtered_scan->begin(), keyframe.filtered_scan->end(),
      [](const auto & point) {return pcl::isFinite(point);}) ||
    !keyframe.front_end_base_pose.matrix().allFinite() ||
    !keyframe.odom_base_pose.matrix().allFinite() ||
    !keyframe.base_to_sensor.matrix().allFinite() ||
    !keyframe.pose_covariance.allFinite() ||
    !std::isfinite(keyframe.accumulated_distance) ||
    keyframe.accumulated_distance < 0.0 || !std::isfinite(keyframe.rmse) ||
    keyframe.rmse < 0.0)
  {
    throw std::invalid_argument("global keyframe fields must be finite and valid");
  }
}

}  // namespace slam_robot_slam_3d
