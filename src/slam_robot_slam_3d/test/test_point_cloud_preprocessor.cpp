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
