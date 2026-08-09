#include <gtest/gtest.h>

#include "slam_robot_slam_3d/se2_pose_graph_backend.hpp"

namespace slam_robot_slam_3d
{
namespace
{

GlobalKeyframe makeKeyframe(std::size_t id, double x)
{
  GlobalKeyframe keyframe;
  keyframe.id = id;
  keyframe.front_end_base_pose.translation().x() = x;
  return keyframe;
}

TEST(Se2PoseGraphBackend, UsesLoopConstraintToCorrectLatestLocalPose)
{
  const std::vector<GlobalKeyframe> keyframes{
    makeKeyframe(0U, 0.0), makeKeyframe(1U, 1.0),
    makeKeyframe(2U, 2.0), makeKeyframe(3U, 3.0)};
  Se2LoopConstraint loop;
  loop.source_id = 0U;
  loop.target_id = 3U;
  const Se2PoseGraphBackend backend(Se2PoseGraphBackendParameters{});

  const auto result = backend.optimize(keyframes, {loop});

  ASSERT_TRUE(result.success);
  EXPECT_GT(result.initial_cost, result.final_cost);
  EXPECT_LT(result.map_from_local.translation().x(), -0.2);
  EXPECT_EQ(result.snapshot_keyframe_count, keyframes.size());
  ASSERT_EQ(result.optimized_base_poses.size(), keyframes.size());
  EXPECT_LT(result.optimized_base_poses.back().translation().x(), 2.8);
}

TEST(Se2PoseGraphBackend, ProjectsMapToOdomCorrectionToSe2)
{
  std::vector<GlobalKeyframe> keyframes{
    makeKeyframe(0U, 0.0), makeKeyframe(1U, 1.0)};
  keyframes.back().odom_base_pose.translation() = Eigen::Vector3d(1.0, 0.0, 0.4);
  keyframes.back().odom_base_pose.linear() =
    (Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ())).toRotationMatrix();
  Se2LoopConstraint loop;
  loop.source_id = 0U;
  loop.target_id = 1U;
  const Se2PoseGraphBackend backend(Se2PoseGraphBackendParameters{});

  const auto result = backend.optimize(keyframes, {loop});

  ASSERT_TRUE(result.success);
  EXPECT_DOUBLE_EQ(result.map_from_odom.translation().z(), 0.0);
  EXPECT_NEAR(result.map_from_odom.rotation()(2, 0), 0.0, 1e-12);
  EXPECT_NEAR(result.map_from_odom.rotation()(2, 1), 0.0, 1e-12);
  EXPECT_NEAR(result.map_from_odom.rotation()(0, 2), 0.0, 1e-12);
  EXPECT_NEAR(result.map_from_odom.rotation()(1, 2), 0.0, 1e-12);
}

}  // namespace
}  // namespace slam_robot_slam_3d
