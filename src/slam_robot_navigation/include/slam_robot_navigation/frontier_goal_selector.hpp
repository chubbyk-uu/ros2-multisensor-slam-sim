// Copyright 2026 Jerry

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <random>
#include <vector>

#include "slam_robot_navigation/frontier_detector.hpp"

namespace slam_robot_navigation
{

struct ScoredFrontierCandidate
{
  FrontierCandidate candidate;
  double score{0.0};
};

struct FrontierGoalSelection
{
  FrontierCandidate candidate;
  double score{0.0};
  std::size_t rank{0U};
  std::size_t pool_size{0U};
};

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
