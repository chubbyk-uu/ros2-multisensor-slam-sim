#include <gtest/gtest.h>

#include "slam_robot_slam_3d/global_map_pose_selector.hpp"

namespace slam_robot_slam_3d
{
namespace
{

GlobalKeyframe makeKeyframe(std::size_t id, double front_end_x)
{
  GlobalKeyframe keyframe;
  keyframe.id = id;
  keyframe.front_end_base_pose.translation().x() = front_end_x;
  // Deliberately different from the scan-matched pose: a keyframe placed from
  // odometry would land here instead, which is how a map used to drift.
  keyframe.odom_base_pose.translation().x() = front_end_x + 5.0;
  return keyframe;
}

Eigen::Isometry3d translation(double x)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().x() = x;
  return pose;
}

}  // namespace

TEST(GlobalMapPoseSelector, OptimisedKeyframesUseTheirOptimisedPose)
{
  const std::vector<GlobalKeyframe> keyframes{
    makeKeyframe(0U, 0.0), makeKeyframe(1U, 1.0)};
  const std::vector<Eigen::Isometry3d> optimized{
    translation(10.0), translation(11.0)};

  const auto poses = selectGlobalMapPoses(
    keyframes, optimized, translation(100.0));

  ASSERT_EQ(poses.size(), 2U);
  // The correction does not apply twice: an optimised pose is already in the
  // map frame.
  EXPECT_DOUBLE_EQ(poses[0].translation().x(), 10.0);
  EXPECT_DOUBLE_EQ(poses[1].translation().x(), 11.0);
}

TEST(GlobalMapPoseSelector, NewerKeyframesCarryTheirScanMatchedPose)
{
  const std::vector<GlobalKeyframe> keyframes{
    makeKeyframe(0U, 0.0), makeKeyframe(1U, 1.0), makeKeyframe(2U, 2.0)};
  const std::vector<Eigen::Isometry3d> optimized{translation(10.0)};

  const auto poses = selectGlobalMapPoses(
    keyframes, optimized, translation(100.0));

  ASSERT_EQ(poses.size(), 3U);
  EXPECT_DOUBLE_EQ(poses[0].translation().x(), 10.0);
  // Front-end pose moved into the map frame, not the odometry pose, which
  // would have put these at 106 and 107.
  EXPECT_DOUBLE_EQ(poses[1].translation().x(), 101.0);
  EXPECT_DOUBLE_EQ(poses[2].translation().x(), 102.0);
}

TEST(GlobalMapPoseSelector, WithoutAnOptimisationEveryPoseComesFromTheFrontEnd)
{
  // The condition that exposed the defect: an exploration pass closing no loop
  // never commits an optimisation, so nothing is ever placed from the prefix.
  const std::vector<GlobalKeyframe> keyframes{
    makeKeyframe(0U, 0.0), makeKeyframe(1U, 1.0)};

  const auto poses = selectGlobalMapPoses(
    keyframes, {}, Eigen::Isometry3d::Identity());

  ASSERT_EQ(poses.size(), 2U);
  EXPECT_DOUBLE_EQ(poses[0].translation().x(), 0.0);
  EXPECT_DOUBLE_EQ(poses[1].translation().x(), 1.0);
}

TEST(GlobalMapPoseSelector, AnEmptySnapshotProducesNoPoses)
{
  EXPECT_TRUE(
    selectGlobalMapPoses({}, {}, Eigen::Isometry3d::Identity()).empty());
}

}  // namespace slam_robot_slam_3d
