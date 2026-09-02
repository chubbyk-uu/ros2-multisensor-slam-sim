#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "slam_robot_slam_3d/point_cloud_preprocessor.hpp"

namespace slam_robot_slam_3d
{
namespace
{

pcl::PointXYZI makePoint(float x, float y, float z, float intensity = 1.0F)
{
  pcl::PointXYZI point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  return point;
}

TEST(PointCloudPreprocessor, RejectsInvalidParameters)
{
  PointCloudPreprocessorParameters parameters;
  parameters.maximum_range = parameters.minimum_range;
  EXPECT_THROW((void)PointCloudPreprocessor{parameters}, std::invalid_argument);

  parameters = PointCloudPreprocessorParameters{};
  parameters.voxel_leaf_size = 0.0;
  EXPECT_THROW((void)PointCloudPreprocessor{parameters}, std::invalid_argument);

  parameters = PointCloudPreprocessorParameters{};
  parameters.self_min_x = parameters.self_max_x;
  EXPECT_THROW((void)PointCloudPreprocessor{parameters}, std::invalid_argument);
}

TEST(PointCloudPreprocessor, RemovesNonFiniteRangeAndSelfReturns)
{
  PointCloudPreprocessorParameters parameters;
  parameters.minimum_range = 0.25;
  parameters.maximum_range = 5.0;
  parameters.voxel_leaf_size = 0.05;
  PointCloudPreprocessor preprocessor(parameters);

  pcl::PointCloud<pcl::PointXYZI> input;
  input.push_back(makePoint(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F));
  input.push_back(makePoint(0.1F, 0.0F, 0.0F));
  input.push_back(makePoint(6.0F, 0.0F, 0.0F));
  input.push_back(makePoint(0.2F, 0.2F, 0.0F));
  input.push_back(makePoint(1.0F, 0.0F, 0.0F, 7.0F));

  PointCloudPreprocessingStatistics statistics;
  const auto output = preprocessor.process(input, &statistics);

  ASSERT_EQ(output.size(), 1U);
  EXPECT_FLOAT_EQ(output.front().x, 1.0F);
  EXPECT_FLOAT_EQ(output.front().intensity, 7.0F);
  EXPECT_EQ(statistics.input_points, 5U);
  EXPECT_EQ(statistics.finite_points, 4U);
  EXPECT_EQ(statistics.range_filtered_points, 2U);
  EXPECT_EQ(statistics.self_filter_passed_points, 1U);
  EXPECT_EQ(statistics.output_points, 1U);
}

TEST(PointCloudPreprocessor, DownsamplesPointsWithinOneVoxel)
{
  PointCloudPreprocessorParameters parameters;
  parameters.minimum_range = 0.0;
  parameters.maximum_range = 10.0;
  parameters.voxel_leaf_size = 0.20;
  parameters.self_filter_enabled = false;
  PointCloudPreprocessor preprocessor(parameters);

  pcl::PointCloud<pcl::PointXYZI> input;
  input.push_back(makePoint(1.01F, 0.01F, 0.01F));
  input.push_back(makePoint(1.05F, 0.02F, 0.02F));
  input.push_back(makePoint(2.0F, 0.0F, 0.0F));

  const auto output = preprocessor.process(input);

  EXPECT_EQ(output.size(), 2U);
  EXPECT_TRUE(output.is_dense);
}

TEST(PointCloudPreprocessor, ExperimentalFiltersAreDisabledByDefault)
{
  PointCloudPreprocessorParameters parameters;
  parameters.minimum_range = 0.0;
  parameters.maximum_range = 10.0;
  parameters.voxel_leaf_size = 0.01;
  parameters.self_filter_enabled = false;
  PointCloudPreprocessor preprocessor(parameters);

  pcl::PointCloud<pcl::PointXYZI> input;
  input.push_back(makePoint(1.0F, 0.0F, -0.30F));
  input.push_back(makePoint(2.0F, 0.0F, 0.40F));
  PointCloudPreprocessingStatistics statistics;

  const auto output = preprocessor.process(input, &statistics);

  EXPECT_EQ(output.size(), 2U);
  EXPECT_EQ(statistics.ground_filter_passed_points, 2U);
  EXPECT_EQ(statistics.outlier_filter_passed_points, 2U);
}

TEST(PointCloudPreprocessor, LowReturnFilterRemovesConfiguredPoints)
{
  PointCloudPreprocessorParameters parameters;
  parameters.minimum_range = 0.0;
  parameters.maximum_range = 10.0;
  parameters.voxel_leaf_size = 0.01;
  parameters.self_filter_enabled = false;
  parameters.ground_filter_enabled = true;
  parameters.ground_filter_maximum_z = -0.20;
  PointCloudPreprocessor preprocessor(parameters);

  pcl::PointCloud<pcl::PointXYZI> input;
  input.push_back(makePoint(1.0F, 0.0F, -0.30F));
  input.push_back(makePoint(2.0F, 0.0F, -0.10F));
  PointCloudPreprocessingStatistics statistics;

  const auto output = preprocessor.process(input, &statistics);

  ASSERT_EQ(output.size(), 1U);
  EXPECT_FLOAT_EQ(output.front().z, -0.10F);
  EXPECT_EQ(statistics.ground_filter_passed_points, 1U);
}

TEST(PointCloudPreprocessor, StatisticalOutlierFilterRemovesAnIsolatedReturn)
{
  PointCloudPreprocessorParameters parameters;
  parameters.minimum_range = 0.0;
  parameters.maximum_range = 20.0;
  parameters.voxel_leaf_size = 0.001;
  parameters.self_filter_enabled = false;
  parameters.outlier_filter_enabled = true;
  parameters.outlier_filter_mean_k = 4;
  parameters.outlier_filter_standard_deviation_multiplier = 1.0;
  PointCloudPreprocessor preprocessor(parameters);

  pcl::PointCloud<pcl::PointXYZI> input;
  for (int x = 0; x < 3; ++x) {
    for (int y = 0; y < 3; ++y) {
      input.push_back(makePoint(
        1.0F + 0.02F * static_cast<float>(x),
        0.02F * static_cast<float>(y), 0.20F));
    }
  }
  input.push_back(makePoint(6.0F, 6.0F, 0.20F));

  PointCloudPreprocessingStatistics statistics;
  const auto output = preprocessor.process(input, &statistics);

  EXPECT_LT(statistics.outlier_filter_passed_points, input.size());
  EXPECT_LT(output.size(), input.size());
  for (const auto & point : output.points) {
    EXPECT_LT(point.x, 2.0F);
  }
}

TEST(PointCloudPreprocessor, RejectsInvalidExperimentalParameters)
{
  PointCloudPreprocessorParameters parameters;
  parameters.ground_filter_maximum_z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((void)PointCloudPreprocessor{parameters}, std::invalid_argument);

  parameters = PointCloudPreprocessorParameters{};
  parameters.outlier_filter_mean_k = 1;
  EXPECT_THROW((void)PointCloudPreprocessor{parameters}, std::invalid_argument);

  parameters = PointCloudPreprocessorParameters{};
  parameters.outlier_filter_standard_deviation_multiplier = 0.0;
  EXPECT_THROW((void)PointCloudPreprocessor{parameters}, std::invalid_argument);
}

TEST(PointCloudPreprocessor, DerivesCoarserRegistrationCloudFromMappingCloud)
{
  pcl::PointCloud<pcl::PointXYZI> mapping_cloud;
  mapping_cloud.push_back(makePoint(1.01F, 0.01F, 0.01F));
  mapping_cloud.push_back(makePoint(1.06F, 0.01F, 0.01F));
  mapping_cloud.push_back(makePoint(1.16F, 0.01F, 0.01F));

  const auto registration_cloud =
    voxelDownsamplePointCloud(mapping_cloud, 0.10);

  EXPECT_LT(registration_cloud.size(), mapping_cloud.size());
  EXPECT_TRUE(registration_cloud.is_dense);
  EXPECT_THROW(
    (void)voxelDownsamplePointCloud(mapping_cloud, 0.0),
    std::invalid_argument);
}

TEST(PointCloudPreprocessor, EmptyInputProducesDenseEmptyOutput)
{
  PointCloudPreprocessor preprocessor(PointCloudPreprocessorParameters{});
  pcl::PointCloud<pcl::PointXYZI> input;
  PointCloudPreprocessingStatistics statistics;

  const auto output = preprocessor.process(input, &statistics);

  EXPECT_TRUE(output.empty());
  EXPECT_TRUE(output.is_dense);
  EXPECT_EQ(statistics.output_points, 0U);
}

}  // namespace
}  // namespace slam_robot_slam_3d
