// Copyright 2026 Jerry

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "slam_robot_navigation/frontier_detector.hpp"

namespace slam_robot_navigation
{

struct ScoredFrontierCandidate
{
  FrontierCandidate candidate;
  double score{0.0};
  double path_length{0.0};
};

struct FrontierGoalSelection
{
  FrontierCandidate candidate;
  double score{0.0};
  std::size_t rank{0U};
  std::size_t pool_size{0U};
  // Position in the list handed to select(), so a record of the decision can
  // mark which entry won without matching on floating-point scores.
  std::size_t index{0U};
};

// Records one goal decision in a form another rule can be replayed against.
//
// The selection rule is a parameter choice, not a fact: a band fraction, a
// softmax temperature and a plain top-k all pick from the same candidate list.
// Tuning any of them from live runs alone costs an hour of machine time per
// value, and the campaign logs used to carry only the pool size and the score
// that won, which is enough to say what happened and not enough to say what
// else would have. Every component of the score is written out so alternative
// rules -- and alternative weights -- can be replayed offline against the same
// decisions.
std::string formatSelectionRecord(
  std::size_t decision, std::size_t candidate_count, std::size_t pool_size,
  std::size_t rank, std::uint64_t seed, double chosen_score);
std::string formatCandidateRecord(
  std::size_t decision, std::size_t index,
  const ScoredFrontierCandidate & scored, bool chosen);
std::string formatCandidateScores(
  const std::vector<ScoredFrontierCandidate> & candidates);

class FrontierGoalSelector
{
public:
  FrontierGoalSelector(double top_score_band_fraction, std::uint64_t random_seed);

  std::optional<FrontierGoalSelection> select(
    const std::vector<ScoredFrontierCandidate> & candidates);

  std::uint64_t effectiveSeed() const;

private:
  double top_score_band_fraction_{0.0};
  std::uint64_t effective_seed_{0U};
  std::mt19937_64 random_engine_;
};

}  // namespace slam_robot_navigation
