#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <pcl/common/transforms.h>

#include "slam_robot_slam_3d/scan_to_map_matcher.hpp"

namespace slam_robot_slam_3d
{
namespace
{

pcl::PointCloud<pcl::PointXYZI> makeStructuredCloud()
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (double first = -1.5; first <= 1.5; first += 0.15) {
    for (double second = -1.0; second <= 1.0; second += 0.15) {
      pcl::PointXYZI floor;
      floor.x = static_cast<float>(first);
      floor.y = static_cast<float>(second);
      floor.z = 0.0F;
      floor.intensity = 1.0F;
      cloud.push_back(floor);

      pcl::PointXYZI wall_x;
      wall_x.x = 2.0F;
      wall_x.y = static_cast<float>(first);
      wall_x.z = static_cast<float>(second + 1.0);
      wall_x.intensity = 2.0F;
      cloud.push_back(wall_x);

      pcl::PointXYZI wall_y;
      wall_y.x = static_cast<float>(first);
      wall_y.y = -1.5F;
      wall_y.z = static_cast<float>(second + 1.0);
      wall_y.intensity = 3.0F;
      cloud.push_back(wall_y);
    }
  }
  cloud.width = cloud.size();
  cloud.height = 1U;
  cloud.is_dense = true;
  return cloud;
}

pcl::PointCloud<pcl::PointXYZI> makeParallelCorridor()
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (double x = -4.0; x <= 4.0; x += 0.10) {
    for (double z = 0.0; z <= 2.0; z += 0.10) {
      for (double y : {-1.0, 1.0}) {
        pcl::PointXYZI point;
        point.x = static_cast<float>(x);
        point.y = static_cast<float>(y);
        point.z = static_cast<float>(z);
        point.intensity = 1.0F;
        cloud.push_back(point);
      }
    }
  }
  cloud.width = cloud.size();
  cloud.height = 1U;
  cloud.is_dense = true;
  return cloud;
}

Eigen::Isometry3d makePose(double x, double y, double z, double yaw)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, z);
  pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

double rotationError(
  const Eigen::Isometry3d & expected, const Eigen::Isometry3d & actual)
{
  return Eigen::AngleAxisd(expected.rotation().transpose() * actual.rotation()).angle();
}

TEST(ScanToMapMatcher, RecoversStructuredCloudPoseFromMotionInitialGuess)
{
  const auto local_map = makeStructuredCloud();
  const auto expected_pose = makePose(0.18, -0.09, 0.03, 0.08);
  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::transformPointCloud(
    local_map, scan, expected_pose.inverse().matrix().cast<float>());

  ScanToMapMatcherParameters parameters;
  parameters.maximum_rmse = 0.05;
  ScanToMapMatcher matcher(parameters);
  const auto initial_pose = makePose(0.13, -0.06, 0.02, 0.05);
  const auto result = matcher.match(scan, local_map, initial_pose);

  ASSERT_TRUE(result.success()) << toString(result.status);
  EXPECT_LT((result.pose.translation() - expected_pose.translation()).norm(), 0.02);
  EXPECT_LT(rotationError(expected_pose, result.pose), 0.02);
  EXPECT_GE(result.correspondence_count, parameters.minimum_correspondences);
  EXPECT_LT(result.rmse, parameters.maximum_rmse);
  EXPECT_GT(result.translation_information_ratio, 0.10);
  EXPECT_GT(result.yaw_information, 0.01);
  EXPECT_FALSE(result.degenerate);
}

TEST(ScanToMapMatcher, DetectsUnobservableCorridorTranslation)
{
  const auto corridor = makeParallelCorridor();
  ScanToMapMatcher matcher(ScanToMapMatcherParameters{});

  const auto result = matcher.match(
    corridor, corridor, Eigen::Isometry3d::Identity());

  ASSERT_TRUE(result.success()) << toString(result.status);
  EXPECT_EQ(result.observability_correspondences, result.correspondence_count);
  EXPECT_LT(result.translation_information_ratio, 0.01);
  EXPECT_GT(result.translation_information_eigenvalues.maxCoeff(), 0.50);
  EXPECT_TRUE(result.degenerate);
  EXPECT_TRUE(result.degeneracy_handling_applied);
  EXPECT_DOUBLE_EQ(result.weak_translation_correction_scale, 0.0);
  EXPECT_GT(std::abs(result.weak_translation_direction.x()), 0.90);
}

TEST(ScanToMapMatcher, RejectsTooFewAndNonFinitePoints)
{
  ScanToMapMatcher matcher(ScanToMapMatcherParameters{});
  pcl::PointCloud<pcl::PointXYZI> small_cloud;
  small_cloud.resize(10U);
  auto result = matcher.match(
    small_cloud, small_cloud, Eigen::Isometry3d::Identity());
  EXPECT_EQ(result.status, ScanToMapStatus::kTooFewPoints);

  auto structured_cloud = makeStructuredCloud();
  structured_cloud.front().x = std::numeric_limits<float>::quiet_NaN();
  result = matcher.match(
    structured_cloud, structured_cloud, Eigen::Isometry3d::Identity());
  EXPECT_EQ(result.status, ScanToMapStatus::kInvalidInput);
}

TEST(ScanToMapMatcher, RejectsCorrectionOutsideMotionPriorGate)
{
  const auto local_map = makeStructuredCloud();
  const auto expected_pose = makePose(0.18, -0.09, 0.03, 0.08);
  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::transformPointCloud(
    local_map, scan, expected_pose.inverse().matrix().cast<float>());

  ScanToMapMatcherParameters parameters;
  parameters.maximum_correction_translation = 0.01;
  parameters.maximum_correction_rotation = 0.01;
  ScanToMapMatcher matcher(parameters);
  const auto result = matcher.match(
    scan, local_map, makePose(0.13, -0.06, 0.02, 0.05));

  EXPECT_EQ(result.status, ScanToMapStatus::kCorrectionTooLarge);
}

TEST(ScanToMapMatcher, RejectsInvalidParameters)
{
  auto parameters = ScanToMapMatcherParameters{};
  parameters.maximum_correspondence_distance = 0.0;
  EXPECT_THROW((void)ScanToMapMatcher{parameters}, std::invalid_argument);

  parameters = ScanToMapMatcherParameters{};
  parameters.minimum_points = 5U;
  EXPECT_THROW((void)ScanToMapMatcher{parameters}, std::invalid_argument);

  parameters = ScanToMapMatcherParameters{};
  parameters.full_suppression_translation_information_ratio =
    parameters.minimum_translation_information_ratio;
  EXPECT_THROW((void)ScanToMapMatcher{parameters}, std::invalid_argument);

  parameters = ScanToMapMatcherParameters{};
  parameters.full_suppression_translation_information =
    parameters.minimum_translation_information;
  EXPECT_THROW((void)ScanToMapMatcher{parameters}, std::invalid_argument);
}

}  // namespace
}  // namespace slam_robot_slam_3d
