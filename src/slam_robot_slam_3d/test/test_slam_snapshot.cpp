#include <filesystem>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "slam_robot_slam_3d/slam_snapshot.hpp"

namespace slam_robot_slam_3d
{
namespace
{
GlobalKeyframe makeKeyframe(std::size_t id, double x)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  cloud->push_back(pcl::PointXYZI{1.0F, 2.0F, 0.3F, 4.0F});
  GlobalKeyframe keyframe;
  keyframe.id = id;
  keyframe.stamp = rclcpp::Time(static_cast<std::int64_t>(1000 + id), RCL_ROS_TIME);
  keyframe.filtered_scan = cloud;
  keyframe.front_end_base_pose.translation().x() = x;
  keyframe.odom_base_pose.translation().x() = x + 0.1;
  keyframe.base_to_sensor.translation().z() = 0.2;
  keyframe.pose_covariance(0, 0) = 0.123;
  keyframe.accumulated_distance = x;
  keyframe.match_accepted = true;
  keyframe.correspondence_count = 42U;
  keyframe.rmse = 0.03;
  return keyframe;
}
}

TEST(SlamSnapshot, RoundTripsAllPersistentState)
{
  const auto path = std::filesystem::temp_directory_path() /
    "slam_robot_slam_3d_snapshot_test.bin";
  SlamSnapshot source;
  source.keyframes = {makeKeyframe(0U, 0.0), makeKeyframe(1U, 1.0)};
  source.loop_constraints.push_back({0U, 1U, Eigen::Isometry3d::Identity()});
  source.optimized_base_poses = {
    Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity()};
  source.optimized_base_poses.back().translation().x() = 0.9;

  saveSlamSnapshot(path.string(), source);
  const auto restored = loadSlamSnapshot(path.string());

  ASSERT_EQ(restored.keyframes.size(), 2U);
  ASSERT_EQ(restored.loop_constraints.size(), 1U);
  ASSERT_EQ(restored.optimized_base_poses.size(), 2U);
  EXPECT_DOUBLE_EQ(restored.keyframes[1].front_end_base_pose.translation().x(), 1.0);
  EXPECT_DOUBLE_EQ(restored.keyframes[1].pose_covariance(0, 0), 0.123);
  EXPECT_EQ(restored.keyframes[1].correspondence_count, 42U);
  EXPECT_FLOAT_EQ(restored.keyframes[0].filtered_scan->front().intensity, 4.0F);
  EXPECT_DOUBLE_EQ(restored.optimized_base_poses[1].translation().x(), 0.9);
  std::filesystem::remove(path);
}

TEST(SlamSnapshot, RejectsEmptyState)
{
  EXPECT_THROW(saveSlamSnapshot("unused", {}), std::invalid_argument);
}
}  // namespace slam_robot_slam_3d
