#include <memory>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "slam_robot_slam_3d/global_point_cloud_map.hpp"

namespace slam_robot_slam_3d
{
namespace
{

GlobalKeyframe makeKeyframe(
  std::size_t id, std::initializer_list<pcl::PointXYZI> points)
{
  auto scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  scan->assign(points.begin(), points.end());
  scan->width = scan->size();
  scan->height = 1U;
  scan->is_dense = true;
  GlobalKeyframe keyframe;
  keyframe.id = id;
  keyframe.registration_scan = scan;
  keyframe.occupancy_scan = scan;
  return keyframe;
}

TEST(GlobalPointCloudMap, ReplaysPosesInBatchesAndDownsamplesAcrossKeyframes)
{
  GlobalPointCloudMap map({0.5, 1U});
  const pcl::PointXYZI origin{0.0F, 0.0F, 0.0F, 1.0F};
  const pcl::PointXYZI duplicate{0.1F, 0.0F, 0.0F, 2.0F};
  const pcl::PointXYZI unique{1.0F, 0.0F, 0.0F, 3.0F};
  std::vector<GlobalKeyframe> keyframes{
    makeKeyframe(0U, {origin, duplicate}), makeKeyframe(1U, {origin, unique})};
  std::vector<Eigen::Isometry3d> poses(2U, Eigen::Isometry3d::Identity());
  poses[1].translation().x() = 1.0;

  map.begin(std::move(keyframes), std::move(poses));
  EXPECT_TRUE(map.active());
  EXPECT_FALSE(map.processBatch());
  EXPECT_EQ(map.processedKeyframes(), 1U);
  EXPECT_TRUE(map.active());
  EXPECT_TRUE(map.processBatch());
  EXPECT_FALSE(map.active());
  EXPECT_EQ(map.totalKeyframes(), 2U);
  ASSERT_EQ(map.cloud().size(), 3U);
  EXPECT_FLOAT_EQ(map.cloud()[0].x, 0.0F);
  EXPECT_FLOAT_EQ(map.cloud()[1].x, 1.0F);
  EXPECT_FLOAT_EQ(map.cloud()[2].x, 2.0F);
}

TEST(GlobalPointCloudMap, RejectsMismatchedOrInvalidSnapshots)
{
  GlobalPointCloudMap map({0.1, 2U});
  EXPECT_THROW(map.begin({}, {}), std::invalid_argument);
  std::vector<GlobalKeyframe> keyframes{
    makeKeyframe(0U, {pcl::PointXYZI{0.0F, 0.0F, 0.0F, 0.0F}})};
  EXPECT_THROW(map.begin(keyframes, {}), std::invalid_argument);
  auto poses = std::vector<Eigen::Isometry3d>(1U, Eigen::Isometry3d::Identity());
  poses.front().translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(map.begin(keyframes, poses), std::invalid_argument);
}

}  // namespace
}  // namespace slam_robot_slam_3d
