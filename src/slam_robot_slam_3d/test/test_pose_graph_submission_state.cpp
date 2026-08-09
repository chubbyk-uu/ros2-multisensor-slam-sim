#include <gtest/gtest.h>

#include "slam_robot_slam_3d/pose_graph_submission_state.hpp"

namespace slam_robot_slam_3d
{
namespace
{

Se2LoopConstraint makeConstraint(std::size_t source, std::size_t target)
{
  Se2LoopConstraint constraint;
  constraint.source_id = source;
  constraint.target_id = target;
  return constraint;
}

TEST(PoseGraphSubmissionState, CommitsVerifiedConstraintsTransactionally)
{
  PoseGraphSubmissionState state(3U);
  state.enqueue({makeConstraint(1U, 10U)});
  const auto task = state.begin(10U, 11U);

  ASSERT_TRUE(task.has_value());
  ASSERT_TRUE(state.taskActive());
  ASSERT_TRUE(state.completeSuccess(task->task_id));
  EXPECT_FALSE(state.taskActive());
  ASSERT_EQ(state.committedConstraints().size(), 1U);
  EXPECT_EQ(state.committedConstraints().front().source_id, 1U);
  EXPECT_EQ(state.pendingConstraintCount(), 0U);
}

TEST(PoseGraphSubmissionState, ReturnsOnlyNewConstraintsAfterFailure)
{
  PoseGraphSubmissionState state(3U);
  state.enqueue({makeConstraint(1U, 10U)});
  const auto committed = state.begin(10U, 11U);
  ASSERT_TRUE(committed.has_value());
  ASSERT_TRUE(state.completeSuccess(committed->task_id));

  state.enqueue({makeConstraint(2U, 20U)});
  const auto failed = state.begin(20U, 21U);
  ASSERT_TRUE(failed.has_value());
  ASSERT_EQ(failed->constraints.size(), 2U);
  ASSERT_TRUE(state.completeFailure(failed->task_id));
  EXPECT_EQ(state.committedConstraints().size(), 1U);
  EXPECT_EQ(state.pendingConstraintCount(), 1U);

  EXPECT_FALSE(state.begin(20U, 21U).has_value());
  const auto retry = state.begin(21U, 22U);
  ASSERT_TRUE(retry.has_value());
  EXPECT_EQ(retry->constraints.size(), 2U);
}

TEST(PoseGraphSubmissionState, DeduplicatesAndRejectsStaleCompletion)
{
  PoseGraphSubmissionState state(1U);
  state.enqueue({makeConstraint(1U, 10U), makeConstraint(1U, 10U)});
  const auto task = state.begin(10U, 11U);

  ASSERT_TRUE(task.has_value());
  EXPECT_EQ(task->constraints.size(), 1U);
  EXPECT_FALSE(state.completeSuccess(task->task_id + 1U));
  EXPECT_TRUE(state.taskActive());
  EXPECT_TRUE(state.completeSuccess(task->task_id));
}

TEST(PoseGraphSubmissionState, RestoresCommittedConstraints)
{
  PoseGraphSubmissionState state(3U);
  state.restoreCommitted({makeConstraint(1U, 10U)}, 10U);
  ASSERT_EQ(state.committedConstraints().size(), 1U);
  state.enqueue({makeConstraint(2U, 14U)});
  const auto task = state.begin(14U, 15U);
  ASSERT_TRUE(task.has_value());
  EXPECT_EQ(task->constraints.size(), 2U);
}

}  // namespace
}  // namespace slam_robot_slam_3d
