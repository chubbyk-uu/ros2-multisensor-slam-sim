#include "slam_robot_slam_3d/global_point_cloud_map.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

#include <pcl/common/point_tests.h>

namespace slam_robot_slam_3d
{

GlobalPointCloudMap::GlobalPointCloudMap(GlobalPointCloudMapParameters parameters)
: parameters_(std::move(parameters))
{
  validateParameters();
}

void GlobalPointCloudMap::begin(
  std::vector<GlobalKeyframe> keyframes,
  std::vector<Eigen::Isometry3d> optimized_base_poses)
{
  if (keyframes.empty() || keyframes.size() != optimized_base_poses.size()) {
    throw std::invalid_argument("global map rebuild snapshot is inconsistent");
  }
  for (std::size_t index = 0U; index < keyframes.size(); ++index) {
    if (keyframes[index].id != index || !keyframes[index].registration_scan ||
      !optimized_base_poses[index].matrix().allFinite())
    {
      throw std::invalid_argument("global map rebuild input is invalid");
    }
  }
  keyframes_ = std::move(keyframes);
  optimized_base_poses_ = std::move(optimized_base_poses);
  next_keyframe_ = 0U;
  cloud_.clear();
  occupied_voxels_.clear();
}

bool GlobalPointCloudMap::processBatch()
{
  if (!active()) {
    return false;
  }
  const std::size_t final_keyframe = std::min(
    keyframes_.size(), next_keyframe_ + parameters_.keyframes_per_batch);
  for (; next_keyframe_ < final_keyframe; ++next_keyframe_) {
    insertScan(keyframes_[next_keyframe_], optimized_base_poses_[next_keyframe_]);
  }
  cloud_.width = cloud_.size();
  cloud_.height = 1U;
  cloud_.is_dense = true;
  return !active();
}

bool GlobalPointCloudMap::active() const
{
  return next_keyframe_ < keyframes_.size();
}

std::size_t GlobalPointCloudMap::processedKeyframes() const
{
  return next_keyframe_;
}

std::size_t GlobalPointCloudMap::totalKeyframes() const
{
  return keyframes_.size();
}

const pcl::PointCloud<pcl::PointXYZI> & GlobalPointCloudMap::cloud() const
{
  return cloud_;
}

bool GlobalPointCloudMap::VoxelIndex::operator==(const VoxelIndex & other) const
{
  return x == other.x && y == other.y && z == other.z;
}

std::size_t GlobalPointCloudMap::VoxelIndexHash::operator()(
  const VoxelIndex & index) const
{
  const auto mix = [](std::uint64_t value) {
      value ^= value >> 30U;
      value *= 0xbf58476d1ce4e5b9ULL;
      value ^= value >> 27U;
      value *= 0x94d049bb133111ebULL;
      return value ^ (value >> 31U);
    };
  return mix(static_cast<std::uint32_t>(index.x)) ^
         (mix(static_cast<std::uint32_t>(index.y)) << 1U) ^
         (mix(static_cast<std::uint32_t>(index.z)) << 2U);
}

void GlobalPointCloudMap::validateParameters() const
{
  if (!std::isfinite(parameters_.voxel_leaf_size) ||
    parameters_.voxel_leaf_size <= 0.0 || parameters_.keyframes_per_batch == 0U)
  {
    throw std::invalid_argument("global point cloud map parameters are invalid");
  }
}

void GlobalPointCloudMap::insertScan(
  const GlobalKeyframe & keyframe,
  const Eigen::Isometry3d & optimized_base_pose)
{
  const Eigen::Isometry3d map_from_sensor =
    optimized_base_pose * keyframe.base_to_sensor;
  const double inverse_leaf_size = 1.0 / parameters_.voxel_leaf_size;
  for (const auto & point : *keyframe.registration_scan) {
    if (!pcl::isFinite(point)) {
      continue;
    }
    const Eigen::Vector3d transformed = map_from_sensor *
      Eigen::Vector3d(point.x, point.y, point.z);
    if (!transformed.allFinite() ||
      transformed.cwiseAbs().maxCoeff() >
      static_cast<double>(std::numeric_limits<float>::max()))
    {
      continue;
    }
    const VoxelIndex index{
      static_cast<int>(std::floor(transformed.x() * inverse_leaf_size)),
      static_cast<int>(std::floor(transformed.y() * inverse_leaf_size)),
      static_cast<int>(std::floor(transformed.z() * inverse_leaf_size))};
    if (!occupied_voxels_.insert(index).second) {
      continue;
    }
    pcl::PointXYZI output = point;
    output.x = static_cast<float>(transformed.x());
    output.y = static_cast<float>(transformed.y());
    output.z = static_cast<float>(transformed.z());
    cloud_.push_back(output);
  }
}

}  // namespace slam_robot_slam_3d
