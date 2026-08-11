#include <filesystem>
#include <fstream>
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

TEST(SlamSnapshot, RoundTripsTheFrontEndFrameCorrection)
{
  // Without this the frame the keyframes were written in is lost, and mapping
  // resumed from the snapshot mixes those poses with new ones expressed in the
  // map frame.
  const auto path = (std::filesystem::temp_directory_path() /
    "slam_robot_slam_3d_snapshot_map_from_local.bin").string();
  SlamSnapshot snapshot;
  snapshot.keyframes.push_back(makeKeyframe(0U, 0.0));
  snapshot.optimized_base_poses.push_back(Eigen::Isometry3d::Identity());
  snapshot.map_from_local.translation() = Eigen::Vector3d(1.5, -2.5, 0.0);

  saveSlamSnapshot(path, snapshot);
  const auto restored = loadSlamSnapshot(path);

  EXPECT_NEAR(restored.map_from_local.translation().x(), 1.5, 1.0e-9);
  EXPECT_NEAR(restored.map_from_local.translation().y(), -2.5, 1.0e-9);
  std::filesystem::remove(path);
}


TEST(SlamSnapshot, RecoversTheFrameCorrectionFromAVersionOneFile)
{
  // Version 1 files exist in users' home directories and the restore path is a
  // documented feature, so they must keep loading. The correction they never
  // stored is recoverable: the last pose entry is that keyframe's pose in the
  // map frame, so composing it with the inverse of the same keyframe's
  // front-end pose recovers the transform between the frames.
  const auto path = (std::filesystem::temp_directory_path() /
    "slam_robot_slam_3d_snapshot_v1.bin").string();
  SlamSnapshot source;
  source.keyframes = {makeKeyframe(0U, 0.0), makeKeyframe(1U, 1.0)};
  source.optimized_base_poses = {
    Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity()};
  source.optimized_base_poses.back().translation() =
    Eigen::Vector3d(4.0, -3.0, 0.0);
  source.map_from_local.translation() = Eigen::Vector3d(9.0, 9.0, 0.0);
  saveSlamSnapshot(path, source);

  // Rewrite the version field in place, then truncate the trailing correction
  // so the file matches what version 1 actually wrote.
  std::fstream file(path, std::ios::binary | std::ios::in | std::ios::out);
  file.seekp(sizeof(std::uint64_t));
  const std::uint32_t one = 1U;
  file.write(reinterpret_cast<const char *>(&one), sizeof(one));
  file.close();
  const auto size = std::filesystem::file_size(path);
  std::filesystem::resize_file(path, size - 16U * sizeof(double));

  const auto restored = loadSlamSnapshot(path);

  // keyframes.back() sits at x = 1.0 and its map pose at x = 4.0, so the
  // frames differ by 3.0 in x and -3.0 in y.
  EXPECT_NEAR(restored.map_from_local.translation().x(), 3.0, 1.0e-9);
  EXPECT_NEAR(restored.map_from_local.translation().y(), -3.0, 1.0e-9);
  std::filesystem::remove(path);
}

}  // namespace slam_robot_slam_3d
