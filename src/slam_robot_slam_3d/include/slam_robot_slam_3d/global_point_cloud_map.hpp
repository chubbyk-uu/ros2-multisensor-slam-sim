#ifndef SLAM_ROBOT_SLAM_3D__GLOBAL_POINT_CLOUD_MAP_HPP_
#define SLAM_ROBOT_SLAM_3D__GLOBAL_POINT_CLOUD_MAP_HPP_

#include <cstddef>
#include <memory>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "slam_robot_slam_3d/global_keyframe_map.hpp"

namespace slam_robot_slam_3d
{

struct GlobalPointCloudMapParameters
{
  double voxel_leaf_size{0.15};
  std::size_t keyframes_per_batch{4U};
};

// Replays immutable scans using one pose per global keyframe.  A caller drives
// processBatch() from a low-priority timer, so rebuilding a loop-corrected map
// never occupies the scan-to-map callback.  Voxel insertion is incremental:
// there is no final all-map filtering pause after a long trajectory.
class GlobalPointCloudMap
{
public:
  explicit GlobalPointCloudMap(GlobalPointCloudMapParameters parameters);

  void begin(
    std::vector<GlobalKeyframe> keyframes,
    std::vector<Eigen::Isometry3d> optimized_base_poses);
  bool processBatch();
  bool active() const;
  std::size_t processedKeyframes() const;
  std::size_t totalKeyframes() const;
  const pcl::PointCloud<pcl::PointXYZI> & cloud() const;

private:
  struct VoxelIndex
  {
    int x{0};
    int y{0};
    int z{0};
    bool operator==(const VoxelIndex & other) const;
  };

  struct VoxelIndexHash
  {
    std::size_t operator()(const VoxelIndex & index) const;
  };

  void validateParameters() const;
  void insertScan(
    const GlobalKeyframe & keyframe,
    const Eigen::Isometry3d & optimized_base_pose);

  GlobalPointCloudMapParameters parameters_;
  std::vector<GlobalKeyframe> keyframes_;
  std::vector<Eigen::Isometry3d> optimized_base_poses_;
  std::size_t next_keyframe_{0U};
  pcl::PointCloud<pcl::PointXYZI> cloud_;
  std::unordered_set<VoxelIndex, VoxelIndexHash> occupied_voxels_;
};

}  // namespace slam_robot_slam_3d

#endif  // SLAM_ROBOT_SLAM_3D__GLOBAL_POINT_CLOUD_MAP_HPP_
