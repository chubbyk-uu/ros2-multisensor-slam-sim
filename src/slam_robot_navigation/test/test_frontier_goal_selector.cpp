// Copyright 2026 Jerry

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "slam_robot_navigation/frontier_goal_selector.hpp"

namespace
{

slam_robot_navigation::ScoredFrontierCandidate scored(double x, double score)
{
  slam_robot_navigation::FrontierCandidate candidate;
  candidate.x = x;
  return {candidate, score};
}

TEST(FrontierGoalSelector, RestrictsRandomChoiceToTopScoreBand)
{
  slam_robot_navigation::FrontierGoalSelector selector(0.25, 42U);
  const std::vector<slam_robot_navigation::ScoredFrontierCandidate> candidates{
    scored(1.0, 10.0), scored(2.0, 9.0), scored(3.0, 0.0)};
  bool selected_first = false;
  bool selected_second = false;
  for (int iteration = 0; iteration < 50; ++iteration) {
    const auto selection = selector.select(candidates);
    ASSERT_TRUE(selection);
    EXPECT_EQ(selection->pool_size, 2U);
    EXPECT_NE(selection->candidate.x, 3.0);
    selected_first = selected_first || selection->candidate.x == 1.0;
    selected_second = selected_second || selection->candidate.x == 2.0;
  }
  EXPECT_TRUE(selected_first);
  EXPECT_TRUE(selected_second);
}

TEST(FrontierGoalSelector, AnIrrelevantWorstCandidateDoesNotWidenTheBand)
{
  // Sizing the band by the span between best and worst let the candidate
  // nobody would drive to decide how many of the others become eligible:
  // {10, 9, 8} admitted only the 10, and adding a 0 admitted all three.
  slam_robot_navigation::FrontierGoalSelector selector(0.05, 3U);
  const std::vector<slam_robot_navigation::ScoredFrontierCandidate> leading{
    scored(1.0, 10.0), scored(2.0, 9.0), scored(3.0, 8.0)};
  std::vector<slam_robot_navigation::ScoredFrontierCandidate> with_a_bad_one =
    leading;
  with_a_bad_one.push_back(scored(4.0, 0.0));

  const auto without = selector.select(leading);
  const auto with = selector.select(with_a_bad_one);

  ASSERT_TRUE(without);
  ASSERT_TRUE(with);
  EXPECT_EQ(without->pool_size, 1U);
  EXPECT_EQ(with->pool_size, without->pool_size);
}

TEST(FrontierGoalSelector, TheBandScalesWithTheBestScoreWhenScoresAreNegative)
{
  // Scores subtract a travel cost, so they can go negative. The band is then
  // measured against the magnitude of the best score, which keeps "within 20%
  // of the best" meaning the same thing on either side of zero.
  slam_robot_navigation::FrontierGoalSelector selector(0.20, 5U);

  const auto selection = selector.select(
    {scored(1.0, -10.0), scored(2.0, -11.5), scored(3.0, -13.0)});

  ASSERT_TRUE(selection);
  // -12.0 is the threshold, so the -13.0 candidate stays out.
  EXPECT_EQ(selection->pool_size, 2U);
  EXPECT_NE(selection->candidate.x, 3.0);
}

TEST(FrontierGoalSelector, ZeroBandAlwaysChoosesTheBestCandidate)
{
  slam_robot_navigation::FrontierGoalSelector selector(0.0, 7U);
  const auto selection = selector.select({scored(1.0, -2.0), scored(2.0, -1.0)});

  ASSERT_TRUE(selection);
  EXPECT_EQ(selection->candidate.x, 2.0);
  EXPECT_EQ(selection->rank, 1U);
  EXPECT_EQ(selection->pool_size, 1U);
}

TEST(FrontierGoalSelector, FixedSeedReproducesTheSelectionSequence)
{
  slam_robot_navigation::FrontierGoalSelector first(1.0, 1234U);
  slam_robot_navigation::FrontierGoalSelector second(1.0, 1234U);
  const std::vector<slam_robot_navigation::ScoredFrontierCandidate> candidates{
    scored(1.0, 3.0), scored(2.0, 2.0), scored(3.0, 1.0)};
  for (int iteration = 0; iteration < 20; ++iteration) {
    const auto first_selection = first.select(candidates);
    const auto second_selection = second.select(candidates);
    ASSERT_TRUE(first_selection);
    ASSERT_TRUE(second_selection);
    EXPECT_EQ(first_selection->candidate.x, second_selection->candidate.x);
  }
}

TEST(FrontierGoalSelector, RejectsInvalidParametersAndScores)
{
  EXPECT_THROW(
    slam_robot_navigation::FrontierGoalSelector(-0.1, 1U), std::invalid_argument);
  EXPECT_THROW(
    slam_robot_navigation::FrontierGoalSelector(1.1, 1U), std::invalid_argument);
  EXPECT_THROW(
    slam_robot_navigation::FrontierGoalSelector(
      std::numeric_limits<double>::quiet_NaN(), 1U),
    std::invalid_argument);

  slam_robot_navigation::FrontierGoalSelector selector(0.2, 1U);
  EXPECT_FALSE(selector.select({}));
  EXPECT_THROW(
    selector.select({scored(1.0, std::numeric_limits<double>::infinity())}),
    std::invalid_argument);
}

TEST(FrontierGoalSelector, AutomaticSeedIsReported)
{
  slam_robot_navigation::FrontierGoalSelector selector(0.2, 0U);
  EXPECT_NE(selector.effectiveSeed(), 0U);
}

}  // namespace
