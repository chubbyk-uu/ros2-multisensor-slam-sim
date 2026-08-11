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

TEST(FrontierGoalSelector, ReportsWhichEntryOfTheListWon)
{
  slam_robot_navigation::FrontierGoalSelector selector(0.0, 11U);

  const auto selection = selector.select(
    {scored(1.0, 4.0), scored(2.0, 9.0), scored(3.0, 6.0)});

  ASSERT_TRUE(selection);
  // Reported as a position, not re-derived by matching the score, so a record
  // of the decision can mark the winner without comparing doubles.
  EXPECT_EQ(selection->index, 1U);
  EXPECT_EQ(selection->candidate.x, 2.0);
}

TEST(FrontierGoalSelector, ARecordedDecisionCarriesEveryScoreComponent)
{
  // The point of the record is replaying a different rule, or different score
  // weights, against the same decision. That needs the inputs to the score,
  // not just the score.
  slam_robot_navigation::FrontierCandidate candidate;
  candidate.x = 4.03;
  candidate.y = 0.93;
  candidate.information_gain = 54.2;
  candidate.clearance = 0.85;
  candidate.robot_distance = 3.12;
  candidate.cluster_cells = 110U;
  const slam_robot_navigation::ScoredFrontierCandidate scored_candidate{
    candidate, 109.21, 3.4};

  const auto line =
    slam_robot_navigation::formatCandidateRecord(7U, 0U, scored_candidate, true);

  EXPECT_EQ(
    line,
    "FRONTIER CANDIDATE decision=7 index=0 x=4.030 y=0.930 score=109.210 "
    "gain=54.200 clearance=0.850 distance=3.120 path=3.400 cells=110 chosen=1");
}

TEST(FrontierGoalSelector, ARecordedDecisionNamesTheRuleThatProducedIt)
{
  const auto line =
    slam_robot_navigation::formatSelectionRecord(7U, 5U, 2U, 1U, 20260811U, 109.21);

  EXPECT_EQ(
    line,
    "FRONTIER SELECTION decision=7 candidates=5 pool=2 rank=1 seed=20260811 "
    "chosen_score=109.210");
}

TEST(FrontierGoalSelector, TheDiagnosticScoreListStaysParseable)
{
  EXPECT_EQ(
    slam_robot_navigation::formatCandidateScores(
      {scored(1.0, 109.21), scored(2.0, 8.9), scored(3.0, -1.5)}),
    "109.210,8.900,-1.500");
  EXPECT_EQ(slam_robot_navigation::formatCandidateScores({}), "");
}

TEST(FrontierGoalSelector, AutomaticSeedIsReported)
{
  slam_robot_navigation::FrontierGoalSelector selector(0.2, 0U);
  EXPECT_NE(selector.effectiveSeed(), 0U);
}

}  // namespace
