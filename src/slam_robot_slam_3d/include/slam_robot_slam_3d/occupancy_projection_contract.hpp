#ifndef SLAM_ROBOT_SLAM_3D__OCCUPANCY_PROJECTION_CONTRACT_HPP_
#define SLAM_ROBOT_SLAM_3D__OCCUPANCY_PROJECTION_CONTRACT_HPP_

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace slam_robot_slam_3d
{

struct OccupancyProjectionContract
{
  double input_voxel_leaf_size{0.05};
  double minimum_obstacle_height{0.05};
  double maximum_obstacle_height{0.45};
  double maximum_ray_range{8.0};
  bool force_planar_motion{true};
};

enum class OccupancyEvidence
{
  kFreeRay,
  kObstacleHit,
  kOutsideRange,
  kAboveObstacleBand,
};

void validateOccupancyProjectionContract(const OccupancyProjectionContract & contract);
bool occupancyProjectionContractsMatch(
  const OccupancyProjectionContract & first,
  const OccupancyProjectionContract & second);
OccupancyEvidence classifyOccupancyEvidence(
  const pcl::PointXYZI & point, const Eigen::Isometry3d & output_from_sensor,
  const OccupancyProjectionContract & contract);
pcl::PointCloud<pcl::PointXYZI> selectPersistentOccupancyEvidence(
  const pcl::PointCloud<pcl::PointXYZI> & input,
  const Eigen::Isometry3d & base_from_sensor,
  const OccupancyProjectionContract & contract);

}  // namespace slam_robot_slam_3d

#endif  // SLAM_ROBOT_SLAM_3D__OCCUPANCY_PROJECTION_CONTRACT_HPP_
