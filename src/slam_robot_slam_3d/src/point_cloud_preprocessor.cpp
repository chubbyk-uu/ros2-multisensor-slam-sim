#include "slam_robot_slam_3d/point_cloud_preprocessor.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include <pcl/common/point_tests.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

namespace slam_robot_slam_3d
{

pcl::PointCloud<pcl::PointXYZI> voxelDownsamplePointCloud(
  const pcl::PointCloud<pcl::PointXYZI> & input, double voxel_leaf_size)
{
  if (!std::isfinite(voxel_leaf_size) || voxel_leaf_size <= 0.0) {
    throw std::invalid_argument("voxel leaf size must be finite and positive");
  }
  pcl::PointCloud<pcl::PointXYZI> output;
  if (!input.empty()) {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    const auto leaf_size = static_cast<float>(voxel_leaf_size);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.setInputCloud(input.makeShared());
    voxel_filter.filter(output);
  }
  output.is_dense = true;
  return output;
}

PointCloudPreprocessor::PointCloudPreprocessor(
  PointCloudPreprocessorParameters parameters)
: parameters_(std::move(parameters))
{
  validateParameters();
}

pcl::PointCloud<pcl::PointXYZI> PointCloudPreprocessor::process(
  const pcl::PointCloud<pcl::PointXYZI> & input,
  PointCloudPreprocessingStatistics * statistics) const
{
  PointCloudPreprocessingStatistics result_statistics;
  result_statistics.input_points = input.size();

  const double minimum_range_squared =
    parameters_.minimum_range * parameters_.minimum_range;
  const double maximum_range_squared =
    parameters_.maximum_range * parameters_.maximum_range;

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
    new pcl::PointCloud<pcl::PointXYZI>());
  filtered->reserve(input.size());
  for (const auto & point : input.points) {
    if (!pcl::isFinite(point)) {
      continue;
    }
    ++result_statistics.finite_points;

    const double range_squared =
      static_cast<double>(point.x) * point.x +
      static_cast<double>(point.y) * point.y +
      static_cast<double>(point.z) * point.z;
    if (
      range_squared < minimum_range_squared ||
      range_squared > maximum_range_squared)
    {
      continue;
    }
    ++result_statistics.range_filtered_points;

    if (parameters_.self_filter_enabled && insideSelfFilterBox(point)) {
      continue;
    }
    ++result_statistics.self_filter_passed_points;

    // This is deliberately a sensor-frame low-return filter, not a general
    // ground segmentation algorithm.  It is only enabled by fixed-bag
    // experiments on the known level-ground robot model.
    if (
      parameters_.ground_filter_enabled &&
      point.z <= parameters_.ground_filter_maximum_z)
    {
      continue;
    }
    ++result_statistics.ground_filter_passed_points;
    filtered->push_back(point);
  }
  filtered->is_dense = true;

  if (parameters_.outlier_filter_enabled && !filtered->empty()) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> outlier_filter;
    outlier_filter.setInputCloud(filtered);
    outlier_filter.setMeanK(parameters_.outlier_filter_mean_k);
    outlier_filter.setStddevMulThresh(
      parameters_.outlier_filter_standard_deviation_multiplier);
    pcl::PointCloud<pcl::PointXYZI>::Ptr denoised(
      new pcl::PointCloud<pcl::PointXYZI>());
    outlier_filter.filter(*denoised);
    denoised->is_dense = true;
    filtered = std::move(denoised);
  }
  result_statistics.outlier_filter_passed_points = filtered->size();

  pcl::PointCloud<pcl::PointXYZI> output =
    voxelDownsamplePointCloud(*filtered, parameters_.voxel_leaf_size);
  result_statistics.output_points = output.size();

  if (statistics != nullptr) {
    *statistics = result_statistics;
  }
  return output;
}

const PointCloudPreprocessorParameters & PointCloudPreprocessor::parameters() const
{
  return parameters_;
}

bool PointCloudPreprocessor::insideSelfFilterBox(
  const pcl::PointXYZI & point) const
{
  return
    point.x >= parameters_.self_min_x &&
    point.x <= parameters_.self_max_x &&
    point.y >= parameters_.self_min_y &&
    point.y <= parameters_.self_max_y &&
    point.z >= parameters_.self_min_z &&
    point.z <= parameters_.self_max_z;
}

void PointCloudPreprocessor::validateParameters() const
{
  if (!std::isfinite(parameters_.minimum_range) || parameters_.minimum_range < 0.0) {
    throw std::invalid_argument("minimum_range must be finite and non-negative");
  }
  if (
    !std::isfinite(parameters_.maximum_range) ||
    parameters_.maximum_range <= parameters_.minimum_range)
  {
    throw std::invalid_argument("maximum_range must be finite and greater than minimum_range");
  }
  if (
    !std::isfinite(parameters_.voxel_leaf_size) ||
    parameters_.voxel_leaf_size <= 0.0)
  {
    throw std::invalid_argument("voxel_leaf_size must be finite and positive");
  }
  const bool finite_self_box =
    std::isfinite(parameters_.self_min_x) &&
    std::isfinite(parameters_.self_max_x) &&
    std::isfinite(parameters_.self_min_y) &&
    std::isfinite(parameters_.self_max_y) &&
    std::isfinite(parameters_.self_min_z) &&
    std::isfinite(parameters_.self_max_z);
  if (
    !finite_self_box ||
    parameters_.self_min_x >= parameters_.self_max_x ||
    parameters_.self_min_y >= parameters_.self_max_y ||
    parameters_.self_min_z >= parameters_.self_max_z)
  {
    throw std::invalid_argument("self-filter bounds must be finite and ordered");
  }
  if (!std::isfinite(parameters_.ground_filter_maximum_z)) {
    throw std::invalid_argument("ground_filter.maximum_z must be finite");
  }
  if (parameters_.outlier_filter_mean_k < 2) {
    throw std::invalid_argument("outlier_filter.mean_k must be at least two");
  }
  if (
    !std::isfinite(parameters_.outlier_filter_standard_deviation_multiplier) ||
    parameters_.outlier_filter_standard_deviation_multiplier <= 0.0)
  {
    throw std::invalid_argument(
      "outlier_filter.standard_deviation_multiplier must be finite and positive");
  }
}

}  // namespace slam_robot_slam_3d
