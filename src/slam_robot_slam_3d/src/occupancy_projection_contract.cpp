#include "slam_robot_slam_3d/occupancy_projection_contract.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <pcl/common/point_tests.h>

namespace slam_robot_slam_3d
{

void validateOccupancyProjectionContract(const OccupancyProjectionContract & contract)
{
  if (!std::isfinite(contract.input_voxel_leaf_size) ||
    contract.input_voxel_leaf_size <= 0.0 ||
    !std::isfinite(contract.minimum_obstacle_height) ||
    !std::isfinite(contract.maximum_obstacle_height) ||
    contract.minimum_obstacle_height < 0.0 ||
    contract.minimum_obstacle_height >= contract.maximum_obstacle_height ||
    !std::isfinite(contract.maximum_ray_range) || contract.maximum_ray_range <= 0.0)
  {
    throw std::invalid_argument("occupancy projection contract is invalid");
  }
}

bool occupancyProjectionContractsMatch(
  const OccupancyProjectionContract & first,
  const OccupancyProjectionContract & second)
{
  const auto close = [](double a, double b) {
      const double scale = std::max({1.0, std::abs(a), std::abs(b)});
      return std::abs(a - b) <= 16.0 * std::numeric_limits<double>::epsilon() * scale;
    };
  return close(first.input_voxel_leaf_size, second.input_voxel_leaf_size) &&
         close(first.minimum_obstacle_height, second.minimum_obstacle_height) &&
         close(first.maximum_obstacle_height, second.maximum_obstacle_height) &&
         close(first.maximum_ray_range, second.maximum_ray_range) &&
         first.force_planar_motion == second.force_planar_motion;
}

OccupancyEvidence classifyOccupancyEvidence(
  const pcl::PointXYZI & point, const Eigen::Isometry3d & output_from_sensor,
  const OccupancyProjectionContract & contract)
{
  if (!pcl::isFinite(point) || !output_from_sensor.matrix().allFinite()) {
    throw std::invalid_argument("occupancy evidence point or pose is invalid");
  }
  const Eigen::Vector3d origin = output_from_sensor.translation();
  const Eigen::Vector3d endpoint =
    output_from_sensor * Eigen::Vector3d(point.x, point.y, point.z);
  if ((endpoint.head<2>() - origin.head<2>()).norm() > contract.maximum_ray_range) {
    return OccupancyEvidence::kOutsideRange;
  }
  if (endpoint.z() > contract.maximum_obstacle_height) {
    return OccupancyEvidence::kAboveObstacleBand;
  }
  return endpoint.z() < contract.minimum_obstacle_height ?
         OccupancyEvidence::kFreeRay : OccupancyEvidence::kObstacleHit;
}

pcl::PointCloud<pcl::PointXYZI> selectPersistentOccupancyEvidence(
  const pcl::PointCloud<pcl::PointXYZI> & input,
  const Eigen::Isometry3d & base_from_sensor,
  const OccupancyProjectionContract & contract)
{
  validateOccupancyProjectionContract(contract);
  pcl::PointCloud<pcl::PointXYZI> output;
  output.reserve(input.size());
  for (const auto & point : input) {
    if (!pcl::isFinite(point)) {
      throw std::invalid_argument("persistent occupancy evidence must be finite");
    }
    if (!contract.force_planar_motion) {
      output.push_back(point);
      continue;
    }
    const auto evidence = classifyOccupancyEvidence(point, base_from_sensor, contract);
    if (evidence == OccupancyEvidence::kFreeRay ||
      evidence == OccupancyEvidence::kObstacleHit)
    {
      output.push_back(point);
    }
  }
  output.is_dense = true;
  return output;
}

}  // namespace slam_robot_slam_3d
