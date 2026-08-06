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
}

}  // namespace
}  // namespace slam_robot_slam_3d
