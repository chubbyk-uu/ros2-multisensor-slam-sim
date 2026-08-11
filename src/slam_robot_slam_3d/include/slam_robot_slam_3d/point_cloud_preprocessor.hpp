#ifndef SLAM_ROBOT_SLAM_3D__POINT_CLOUD_PREPROCESSOR_HPP_
#define SLAM_ROBOT_SLAM_3D__POINT_CLOUD_PREPROCESSOR_HPP_

#include <cstddef>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace slam_robot_slam_3d
{

struct PointCloudPreprocessorParameters
{
  double minimum_range{0.25};
  double maximum_range{20.0};
  double voxel_leaf_size{0.05};
  bool self_filter_enabled{true};
  double self_min_x{-0.30};
  double self_max_x{0.30};
  double self_min_y{-0.23};
  double self_max_y{0.23};
  double self_min_z{-0.30};
  double self_max_z{0.05};
};

struct PointCloudPreprocessingStatistics
{
  std::size_t input_points{0U};
  std::size_t finite_points{0U};
  std::size_t range_filtered_points{0U};
  std::size_t self_filter_passed_points{0U};
  std::size_t output_points{0U};
};

pcl::PointCloud<pcl::PointXYZI> voxelDownsamplePointCloud(
  const pcl::PointCloud<pcl::PointXYZI> & input, double voxel_leaf_size);

class PointCloudPreprocessor
{
public:
  explicit PointCloudPreprocessor(
    PointCloudPreprocessorParameters parameters);

  pcl::PointCloud<pcl::PointXYZI> process(
    const pcl::PointCloud<pcl::PointXYZI> & input,
    PointCloudPreprocessingStatistics * statistics = nullptr) const;

  const PointCloudPreprocessorParameters & parameters() const;

private:
  bool insideSelfFilterBox(const pcl::PointXYZI & point) const;
  void validateParameters() const;

  PointCloudPreprocessorParameters parameters_;
};

}  // namespace slam_robot_slam_3d

#endif  // SLAM_ROBOT_SLAM_3D__POINT_CLOUD_PREPROCESSOR_HPP_
