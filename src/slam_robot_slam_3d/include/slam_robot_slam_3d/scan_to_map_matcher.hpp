#ifndef SLAM_ROBOT_SLAM_3D__SCAN_TO_MAP_MATCHER_HPP_
#define SLAM_ROBOT_SLAM_3D__SCAN_TO_MAP_MATCHER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>

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
  double minimum_translation_information_ratio{0.05};
  double full_suppression_translation_information_ratio{0.01};
  double minimum_translation_information{0.01};
  double full_suppression_translation_information{0.002};
  double minimum_planar_information{0.01};
  double minimum_yaw_information{0.10};
  bool degeneracy_handling_enabled{true};
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
  double applied_correction_translation{0.0};
  double applied_correction_rotation{0.0};
  std::size_t observability_correspondences{0U};
  Eigen::Vector2d translation_information_eigenvalues{
    Eigen::Vector2d::Zero()};
  Eigen::Vector2d weak_translation_direction{Eigen::Vector2d::UnitX()};
  Eigen::Vector3d planar_information_eigenvalues{
    Eigen::Vector3d::Zero()};
  double translation_information_ratio{0.0};
  double yaw_information{0.0};
  int translation_observable_rank{0};
  bool translation_degenerate{true};
  bool planar_degenerate{true};
  bool yaw_degenerate{true};
  bool degenerate{true};
  bool degeneracy_handling_applied{false};
  double weak_translation_correction_scale{1.0};
  bool target_cache_reused{false};

  bool success() const;
  Eigen::Matrix2d translationCovariance(
    double nominal_variance, double unobservable_variance) const;
};

class ScanToMapMatcher
{
public:
  explicit ScanToMapMatcher(ScanToMapMatcherParameters parameters);
  ~ScanToMapMatcher();

  ScanToMapMatcher(const ScanToMapMatcher &) = delete;
  ScanToMapMatcher & operator=(const ScanToMapMatcher &) = delete;

  ScanToMapResult match(
    const pcl::PointCloud<pcl::PointXYZI> & scan,
    const pcl::PointCloud<pcl::PointXYZI> & local_map,
    std::uint64_t local_map_version,
    const Eigen::Isometry3d & initial_pose);

  const ScanToMapMatcherParameters & parameters() const;

private:
  class Impl;

  void validateParameters() const;

  ScanToMapMatcherParameters parameters_;
  std::unique_ptr<Impl> implementation_;
};

const char * toString(ScanToMapStatus status);

}  // namespace slam_robot_slam_3d

#endif  // SLAM_ROBOT_SLAM_3D__SCAN_TO_MAP_MATCHER_HPP_
