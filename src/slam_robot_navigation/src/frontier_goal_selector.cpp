// Copyright 2026 Jerry

#include "slam_robot_navigation/frontier_goal_selector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace slam_robot_navigation
{
namespace
{

std::uint64_t makeRandomSeed()
{
  std::random_device device;
  const auto time = static_cast<std::uint64_t>(
    std::chrono::steady_clock::now().time_since_epoch().count());
  const std::uint64_t seed = (static_cast<std::uint64_t>(device()) << 32U) ^
    static_cast<std::uint64_t>(device()) ^ time;
  return seed == 0U ? 1U : seed;
}

}  // namespace

FrontierGoalSelector::FrontierGoalSelector(
  double top_score_band_fraction, std::uint64_t random_seed)
: top_score_band_fraction_(top_score_band_fraction),
  effective_seed_(random_seed == 0U ? makeRandomSeed() : random_seed),
  random_engine_(effective_seed_)
{
  if (!std::isfinite(top_score_band_fraction_) || top_score_band_fraction_ < 0.0 ||
    top_score_band_fraction_ > 1.0)
  {
    throw std::invalid_argument("top score band fraction must be in [0, 1]");
  }
}

std::optional<FrontierGoalSelection> FrontierGoalSelector::select(
  const std::vector<ScoredFrontierCandidate> & candidates)
{
  if (candidates.empty()) {return std::nullopt;}
  const auto score_order = [](const auto & lhs, const auto & rhs) {
      return lhs.score < rhs.score;
    };
  if (std::any_of(candidates.begin(), candidates.end(), [](const auto & candidate) {
      return !std::isfinite(candidate.score);
    }))
  {
    throw std::invalid_argument("frontier candidate scores must be finite");
  }

  const auto [worst, best] = std::minmax_element(
    candidates.begin(), candidates.end(), score_order);
  const double score_span = best->score - worst->score;
  const double threshold = best->score - top_score_band_fraction_ * score_span;
  std::vector<std::size_t> pool;
  pool.reserve(candidates.size());
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    if (candidates[index].score >= threshold) {pool.push_back(index);}
  }
  if (pool.empty()) {
    throw std::logic_error("near-best frontier candidate pool is empty");
  }

  std::uniform_int_distribution<std::size_t> distribution(0U, pool.size() - 1U);
  const auto & selected = candidates[pool[distribution(random_engine_)]];
  const std::size_t rank = 1U + static_cast<std::size_t>(std::count_if(
      candidates.begin(), candidates.end(), [&selected](const auto & candidate) {
        return candidate.score > selected.score;
      }));
  return FrontierGoalSelection{selected.candidate, selected.score, rank, pool.size()};
}

std::uint64_t FrontierGoalSelector::effectiveSeed() const
{
  return effective_seed_;
}

}  // namespace slam_robot_navigation
