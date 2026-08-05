#include <algorithm>
#include <limits>

#include <gtest/gtest.h>

#include "slam_robot_slam_3d/local_submap.hpp"

namespace slam_robot_slam_3d
{
namespace
{

pcl::PointCloud<pcl::PointXYZI> makeCloud(double offset)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (int index = 0; index < 20; ++index) {
    pcl::PointXYZI point;
    point.x = static_cast<float>(offset + 0.2 * index);
    point.y = static_cast<float>(0.1 * (index % 3));
    point.z = static_cast<float>(0.1 * (index % 5));
    point.intensity = static_cast<float>(index);
    cloud.push_back(point);
  }
  cloud.width = cloud.size();
  cloud.height = 1U;
  cloud.is_dense = true;
  return cloud;
}

TEST(LocalSubmap, TransformsAndBoundsKeyframes)
{
  LocalSubmapParameters parameters;
  parameters.maximum_keyframes = 2U;
  parameters.voxel_leaf_size = 0.05;
  LocalSubmap submap(parameters);
  EXPECT_EQ(submap.version(), 0U);

  Eigen::Isometry3d first_pose = Eigen::Isometry3d::Identity();
  first_pose.translation().x() = 1.0;
  submap.addKeyframe(makeCloud(0.0), first_pose);
  EXPECT_EQ(submap.version(), 1U);
  EXPECT_EQ(submap.keyframeCount(), 1U);
  ASSERT_FALSE(submap.cloud().empty());
  EXPECT_GE(submap.cloud().front().x, 1.0F);

  submap.addKeyframe(makeCloud(5.0), Eigen::Isometry3d::Identity());
  submap.addKeyframe(makeCloud(10.0), Eigen::Isometry3d::Identity());
  EXPECT_EQ(submap.version(), 3U);
  EXPECT_EQ(submap.keyframeCount(), 2U);
  EXPECT_TRUE(std::all_of(
    submap.cloud().begin(), submap.cloud().end(),
      [](const auto & point) {return point.x >= 5.0F;}));
}

TEST(LocalSubmap, ClearRemovesCloudAndKeyframes)
{
  LocalSubmap submap(LocalSubmapParameters{});
  submap.addKeyframe(makeCloud(0.0), Eigen::Isometry3d::Identity());
  const auto populated_version = submap.version();
  submap.clear();
  EXPECT_EQ(submap.version(), populated_version + 1U);
  EXPECT_EQ(submap.keyframeCount(), 0U);
  EXPECT_TRUE(submap.cloud().empty());
  EXPECT_TRUE(submap.cloud().is_dense);
}

TEST(LocalSubmap, RejectsInvalidParametersAndKeyframes)
{
  auto parameters = LocalSubmapParameters{};
  parameters.maximum_keyframes = 0U;
  EXPECT_THROW((void)LocalSubmap{parameters}, std::invalid_argument);

  LocalSubmap submap(LocalSubmapParameters{});
  pcl::PointCloud<pcl::PointXYZI> empty;
  EXPECT_THROW(
    submap.addKeyframe(empty, Eigen::Isometry3d::Identity()),
    std::invalid_argument);

  auto invalid_cloud = makeCloud(0.0);
  invalid_cloud.front().z = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(
    submap.addKeyframe(invalid_cloud, Eigen::Isometry3d::Identity()),
    std::invalid_argument);
}

}  // namespace
}  // namespace slam_robot_slam_3d
