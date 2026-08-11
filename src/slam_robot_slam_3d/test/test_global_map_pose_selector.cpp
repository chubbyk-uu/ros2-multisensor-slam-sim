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


TEST(GlobalMapPoseSelector, RebasingLeavesEveryHistoricalEdgeUnchanged)
{
  // The pose graph builds edges from differences between consecutive
  // front-end poses, so a rigid rebase must not disturb any of them.
  std::vector<GlobalKeyframe> keyframes{
    makeKeyframe(0U, 0.0), makeKeyframe(1U, 1.0), makeKeyframe(2U, 2.5)};
  std::vector<double> before;
  for (std::size_t index = 1U; index < keyframes.size(); ++index) {
    before.push_back(
      (keyframes[index - 1U].front_end_base_pose.inverse() *
      keyframes[index].front_end_base_pose).translation().x());
  }

  Eigen::Isometry3d map_from_local = translation(40.0);
  map_from_local.linear() =
    Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  rebaseFrontEndPoses(keyframes, map_from_local);

  for (std::size_t index = 1U; index < keyframes.size(); ++index) {
    EXPECT_NEAR(
      (keyframes[index - 1U].front_end_base_pose.inverse() *
      keyframes[index].front_end_base_pose).translation().x(),
      before[index - 1U], 1.0e-9);
  }
}

TEST(GlobalMapPoseSelector, RebasingJoinsResumedMappingWithoutAJump)
{
  // The edge that only exists after a restart: last restored keyframe to first
  // new one. A new keyframe is created at the resumed front-end pose, which is
  // the last optimised pose, so the join must span the small distance actually
  // travelled rather than the whole accumulated correction.
  std::vector<GlobalKeyframe> restored{
    makeKeyframe(0U, 0.0), makeKeyframe(1U, 1.0)};
  const Eigen::Isometry3d map_from_local = translation(40.0);
  const Eigen::Isometry3d resumed_pose =
    map_from_local * restored.back().front_end_base_pose;

  rebaseFrontEndPoses(restored, map_from_local);
  GlobalKeyframe first_new = makeKeyframe(2U, 0.0);
  first_new.front_end_base_pose = resumed_pose;
  first_new.front_end_base_pose.translation().x() += 0.25;

  const double join =
    (restored.back().front_end_base_pose.inverse() *
    first_new.front_end_base_pose).translation().x();

  EXPECT_NEAR(join, 0.25, 1.0e-9);
}

}  // namespace slam_robot_slam_3d
