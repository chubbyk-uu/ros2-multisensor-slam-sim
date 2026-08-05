#ifndef SLAM_ROBOT_SLAM_3D__SCAN_TO_MAP_MATCHER_HPP_
#define SLAM_ROBOT_SLAM_3D__SCAN_TO_MAP_MATCHER_HPP_

#include <cstddef>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace slam_robot_slam_3d
{

struct ScanToMapMatcherParameters
{
  int maximum_iterations{30};
  int maximum_optimizer_iterations{20};
  int correspondence_randomness{20};
  double maximum_correspondence_distance{0.50};
  double transformation_epsilon{1.0e-4};
  double rotation_epsilon{1.0e-4};
  double euclidean_fitness_epsilon{1.0e-4};
  std::size_t minimum_points{100U};
  std::size_t minimum_correspondences{80U};
  double maximum_rmse{0.15};
  double maximum_correction_translation{0.60};
  double maximum_correction_rotation{0.40};
};

enum class ScanToMapStatus
{
  kSuccess,
  kInvalidInput,
  kTooFewPoints,
  kNotConverged,
  kInsufficientCorrespondences,
  kFitnessTooHigh,
  kCorrectionTooLarge,
};

struct ScanToMapResult
{
  ScanToMapStatus status{ScanToMapStatus::kInvalidInput};
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  std::size_t correspondence_count{0U};
  double rmse{0.0};
  double correction_translation{0.0};
  double correction_rotation{0.0};

  bool success() const;
};

class ScanToMapMatcher
{
public:
  explicit ScanToMapMatcher(ScanToMapMatcherParameters parameters);

  ScanToMapResult match(
    const pcl::PointCloud<pcl::PointXYZI> & scan,
    const pcl::PointCloud<pcl::PointXYZI> & local_map,
    const Eigen::Isometry3d & initial_pose) const;

  const ScanToMapMatcherParameters & parameters() const;

private:
  void validateParameters() const;

  ScanToMapMatcherParameters parameters_;
};

const char * toString(ScanToMapStatus status);

}  // namespace slam_robot_slam_3d

#endif  // SLAM_ROBOT_SLAM_3D__SCAN_TO_MAP_MATCHER_HPP_
