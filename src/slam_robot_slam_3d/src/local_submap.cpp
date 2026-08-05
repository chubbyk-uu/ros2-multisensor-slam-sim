#include "slam_robot_slam_3d/local_submap.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace slam_robot_slam_3d
{

LocalSubmap::LocalSubmap(LocalSubmapParameters parameters)
: parameters_(std::move(parameters))
{
  validateParameters();
}

void LocalSubmap::addKeyframe(
  const pcl::PointCloud<pcl::PointXYZI> & scan,
  const Eigen::Isometry3d & scan_pose)
{
  if (!scan_pose.matrix().allFinite() ||
    !std::all_of(
      scan.begin(), scan.end(),
      [](const auto & point) {return pcl::isFinite(point);}))
  {
    throw std::invalid_argument("keyframe scan and pose must be finite");
  }
  if (scan.empty()) {
    throw std::invalid_argument("keyframe scan must not be empty");
  }

  pcl::PointCloud<pcl::PointXYZI> transformed;
  pcl::transformPointCloud(scan, transformed, scan_pose.matrix().cast<float>());
  transformed.is_dense = true;
  keyframes_.push_back(std::move(transformed));
  while (keyframes_.size() > parameters_.maximum_keyframes) {
    keyframes_.pop_front();
  }
  rebuild();
  ++version_;
}

void LocalSubmap::clear()
{
  keyframes_.clear();
  cloud_.clear();
  cloud_.is_dense = true;
  ++version_;
}

const pcl::PointCloud<pcl::PointXYZI> & LocalSubmap::cloud() const
{
  return cloud_;
}

std::size_t LocalSubmap::keyframeCount() const
{
  return keyframes_.size();
}

std::uint64_t LocalSubmap::version() const
{
  return version_;
}

const LocalSubmapParameters & LocalSubmap::parameters() const
{
  return parameters_;
}

void LocalSubmap::rebuild()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr combined(
    new pcl::PointCloud<pcl::PointXYZI>());
  std::size_t total_points = 0U;
  for (const auto & keyframe : keyframes_) {
    total_points += keyframe.size();
  }
  combined->reserve(total_points);
  for (const auto & keyframe : keyframes_) {
    *combined += keyframe;
  }

  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  const auto leaf_size = static_cast<float>(parameters_.voxel_leaf_size);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.setInputCloud(combined);
  voxel_filter.filter(cloud_);
  cloud_.is_dense = true;
}

void LocalSubmap::validateParameters() const
{
  if (parameters_.maximum_keyframes == 0U) {
    throw std::invalid_argument("maximum_keyframes must be positive");
  }
  if (!std::isfinite(parameters_.voxel_leaf_size) ||
    parameters_.voxel_leaf_size <= 0.0)
  {
    throw std::invalid_argument("voxel_leaf_size must be finite and positive");
  }
}

}  // namespace slam_robot_slam_3d
