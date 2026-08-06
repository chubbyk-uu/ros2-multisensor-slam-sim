#include <cstdint>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "slam_robot_slam_3d/global_keyframe_map.hpp"

namespace slam_robot_slam_3d
{
namespace
{

GlobalKeyframe makeKeyframe(double offset)
{
  auto scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int index = 0; index < 3; ++index) {
    pcl::PointXYZI point;
    point.x = static_cast<float>(offset + index);
    point.y = static_cast<float>(index);
    point.z = 0.1F;
    point.intensity = static_cast<float>(index);
    scan->push_back(point);
  }
  scan->width = scan->size();
  scan->height = 1U;
  scan->is_dense = true;

  GlobalKeyframe keyframe;
  keyframe.stamp = rclcpp::Time(std::int64_t{123456789}, RCL_ROS_TIME);
  keyframe.filtered_scan = scan;
  keyframe.front_end_base_pose.translation().x() = offset;
  keyframe.odom_base_pose.translation().x() = offset + 0.1;
  keyframe.accumulated_distance = offset;
  keyframe.match_accepted = true;
  keyframe.correspondence_count = 100U;
  keyframe.rmse = 0.02;
  return keyframe;
}

TEST(GlobalKeyframeMap, AssignsIdsAndPreservesImmutableSnapshotData)
{
  GlobalKeyframeMap map;
  EXPECT_EQ(map.add(makeKeyframe(0.0)), 0U);
  EXPECT_EQ(map.add(makeKeyframe(2.0)), 1U);
  EXPECT_EQ(map.size(), 2U);
  EXPECT_EQ(map.pointCount(), 6U);

  const auto snapshot = map.snapshot();
  ASSERT_EQ(snapshot.size(), 2U);
  EXPECT_EQ(snapshot[0].id, 0U);
  EXPECT_EQ(snapshot[1].id, 1U);
  EXPECT_TRUE(snapshot[1].match_accepted);
  EXPECT_EQ(snapshot[1].correspondence_count, 100U);
  EXPECT_DOUBLE_EQ(snapshot[1].front_end_base_pose.translation().x(), 2.0);
  EXPECT_DOUBLE_EQ(snapshot[1].odom_base_pose.translation().x(), 2.1);
  EXPECT_EQ(snapshot[0].filtered_scan.use_count(), 2L);
}

TEST(GlobalKeyframeMap, RejectsInvalidData)
{
  GlobalKeyframeMap map;
  auto empty_scan = makeKeyframe(0.0);
  std::const_pointer_cast<pcl::PointCloud<pcl::PointXYZI>>(
    empty_scan.filtered_scan)->clear();
  EXPECT_THROW(map.add(std::move(empty_scan)), std::invalid_argument);

  auto invalid_pose = makeKeyframe(0.0);
  invalid_pose.front_end_base_pose.translation().x() =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(map.add(std::move(invalid_pose)), std::invalid_argument);

  auto invalid_scan = makeKeyframe(0.0);
  auto mutable_scan = std::const_pointer_cast<pcl::PointCloud<pcl::PointXYZI>>(
    invalid_scan.filtered_scan);
  mutable_scan->front().z = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(map.add(std::move(invalid_scan)), std::invalid_argument);
}

}  // namespace
}  // namespace slam_robot_slam_3d
