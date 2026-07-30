#include "slam_robot_slam/correlative_scan_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace slam_robot_slam
{
namespace
{

class CorrelationGrid
{
public:
  CorrelationGrid(
    const std::vector<Point2D> & points,
    const double resolution,
    const double smear_deviation)
  : resolution_(resolution)
  {
    const auto x_limits = std::minmax_element(
      points.begin(), points.end(),
      [](const Point2D & first, const Point2D & second) {
        return first.x < second.x;
      });
    const auto y_limits = std::minmax_element(
      points.begin(), points.end(),
      [](const Point2D & first, const Point2D & second) {
        return first.y < second.y;
      });

    const double padding = 3.0 * smear_deviation + 2.0 * resolution;
    origin_x_ = static_cast<double>(x_limits.first->x) - padding;
    origin_y_ = static_cast<double>(y_limits.first->y) - padding;
    const double maximum_x =
      static_cast<double>(x_limits.second->x) + padding;
    const double maximum_y =
      static_cast<double>(y_limits.second->y) + padding;
    width_ = static_cast<std::size_t>(
      std::ceil((maximum_x - origin_x_) / resolution_)) + 1U;
    height_ = static_cast<std::size_t>(
      std::ceil((maximum_y - origin_y_) / resolution_)) + 1U;
    cells_.assign(width_ * height_, 0.0F);

    std::vector<uint8_t> occupied(width_ * height_, 0U);
    for (const auto & point : points) {
      const int grid_x = worldToGridX(point.x);
      const int grid_y = worldToGridY(point.y);
      if (contains(grid_x, grid_y)) {
        occupied[
          static_cast<std::size_t>(grid_y) * width_ +
          static_cast<std::size_t>(grid_x)] = 1U;
      }
    }

    const int smear_cells =
      static_cast<int>(std::ceil(3.0 * smear_deviation / resolution_));
    const double two_variance =
      2.0 * smear_deviation * smear_deviation;
    const int kernel_width = 2 * smear_cells + 1;
    std::vector<float> kernel(
      static_cast<std::size_t>(kernel_width * kernel_width), 0.0F);
    for (int offset_y = -smear_cells; offset_y <= smear_cells; ++offset_y) {
      for (int offset_x = -smear_cells; offset_x <= smear_cells; ++offset_x) {
        const double distance_squared =
          static_cast<double>(offset_x * offset_x + offset_y * offset_y) *
          resolution_ * resolution_;
        kernel[
          static_cast<std::size_t>(offset_y + smear_cells) *
          static_cast<std::size_t>(kernel_width) +
          static_cast<std::size_t>(offset_x + smear_cells)] =
          static_cast<float>(std::exp(-distance_squared / two_variance));
      }
    }

    for (std::size_t center_y = 0U; center_y < height_; ++center_y) {
      for (std::size_t center_x = 0U; center_x < width_; ++center_x) {
        if (occupied[center_y * width_ + center_x] == 0U) {
          continue;
        }
        for (int offset_y = -smear_cells;
          offset_y <= smear_cells; ++offset_y)
        {
          for (int offset_x = -smear_cells;
            offset_x <= smear_cells; ++offset_x)
          {
            const int grid_x = static_cast<int>(center_x) + offset_x;
            const int grid_y = static_cast<int>(center_y) + offset_y;
            if (!contains(grid_x, grid_y)) {
              continue;
            }
            const float likelihood = kernel[
              static_cast<std::size_t>(offset_y + smear_cells) *
              static_cast<std::size_t>(kernel_width) +
              static_cast<std::size_t>(offset_x + smear_cells)];
            auto & cell = cells_[
              static_cast<std::size_t>(grid_y) * width_ +
              static_cast<std::size_t>(grid_x)];
            cell = std::max(cell, likelihood);
          }
        }
      }
    }
  }

  float likelihood(const Point2D & point) const
  {
    const int grid_x = worldToGridX(point.x);
    const int grid_y = worldToGridY(point.y);
    if (!contains(grid_x, grid_y)) {
      return 0.0F;
    }
    return cells_[
      static_cast<std::size_t>(grid_y) * width_ +
      static_cast<std::size_t>(grid_x)];
  }

private:
  int worldToGridX(const double x) const
  {
    return static_cast<int>(std::lround((x - origin_x_) / resolution_));
  }

  int worldToGridY(const double y) const
  {
    return static_cast<int>(std::lround((y - origin_y_) / resolution_));
  }

  bool contains(const int x, const int y) const
  {
    return x >= 0 && y >= 0 &&
           static_cast<std::size_t>(x) < width_ &&
           static_cast<std::size_t>(y) < height_;
  }

  double resolution_;
  double origin_x_{0.0};
  double origin_y_{0.0};
  std::size_t width_{0U};
  std::size_t height_{0U};
  std::vector<float> cells_;
};

struct CandidateScore
{
  Pose2D pose;
  double score{-std::numeric_limits<double>::infinity()};
  std::size_t matched_points{0U};
};

CandidateScore scoreCandidate(
  const CorrelationGrid & grid,
  const std::vector<Point2D> & current_points,
  const Pose2D & candidate,
  const Pose2D & predicted_pose,
  const CorrelativeScanMatcherParameters & parameters)
{
  double likelihood_sum = 0.0;
  std::size_t matched_points = 0U;
  const double cosine = std::cos(candidate.yaw);
  const double sine = std::sin(candidate.yaw);
  for (const auto & point : current_points) {
    const Point2D transformed{
      static_cast<float>(
        cosine * static_cast<double>(point.x) -
        sine * static_cast<double>(point.y) + candidate.x),
      static_cast<float>(
        sine * static_cast<double>(point.x) +
        cosine * static_cast<double>(point.y) + candidate.y)};
    const float likelihood = grid.likelihood(transformed);
    likelihood_sum += static_cast<double>(likelihood);
    if (likelihood > 0.01F) {
      ++matched_points;
    }
  }

  const double raw_score =
    likelihood_sum / static_cast<double>(current_points.size());
  const double translation_offset =
    std::hypot(
    candidate.x - predicted_pose.x,
    candidate.y - predicted_pose.y);
  const double rotation_offset =
    std::abs(normalizeAngle(candidate.yaw - predicted_pose.yaw));
  const double normalized_translation =
    parameters.linear_search_window > 0.0 ?
    translation_offset / parameters.linear_search_window : 0.0;
  const double normalized_rotation =
    parameters.angular_search_window > 0.0 ?
    rotation_offset / parameters.angular_search_window : 0.0;
  const double penalty = std::exp(
    -parameters.translation_penalty_weight *
    normalized_translation * normalized_translation -
    parameters.rotation_penalty_weight *
    normalized_rotation * normalized_rotation);
  return CandidateScore{
    candidate,
    raw_score * penalty,
    matched_points};
}

CandidateScore searchWindow(
  const CorrelationGrid & grid,
  const std::vector<Point2D> & current_points,
  const Pose2D & center,
  const Pose2D & predicted_pose,
  const double linear_window,
  const double angular_window,
  const double linear_resolution,
  const double angular_resolution,
  const CorrelativeScanMatcherParameters & parameters,
  std::size_t & evaluated_candidates)
{
  CandidateScore best;
  const int linear_steps =
    static_cast<int>(std::ceil(linear_window / linear_resolution));
  const int angular_steps =
    static_cast<int>(std::ceil(angular_window / angular_resolution));

  for (int yaw_step = -angular_steps;
    yaw_step <= angular_steps; ++yaw_step)
  {
    for (int y_step = -linear_steps; y_step <= linear_steps; ++y_step) {
      for (int x_step = -linear_steps; x_step <= linear_steps; ++x_step) {
        const Pose2D candidate{
          center.x + static_cast<double>(x_step) * linear_resolution,
          center.y + static_cast<double>(y_step) * linear_resolution,
          normalizeAngle(
            center.yaw +
            static_cast<double>(yaw_step) * angular_resolution)};
        const CandidateScore scored = scoreCandidate(
          grid, current_points, candidate, predicted_pose, parameters);
        ++evaluated_candidates;
        if (scored.score > best.score) {
          best = scored;
        }
      }
    }
  }
  return best;
}

}  // namespace

void validateCorrelativeScanMatcherParameters(
  const CorrelativeScanMatcherParameters & parameters)
{
  if (!std::isfinite(parameters.grid_resolution) ||
    !std::isfinite(parameters.smear_deviation) ||
    !std::isfinite(parameters.linear_search_window) ||
    !std::isfinite(parameters.angular_search_window) ||
    !std::isfinite(parameters.coarse_linear_resolution) ||
    !std::isfinite(parameters.coarse_angular_resolution) ||
    !std::isfinite(parameters.fine_linear_resolution) ||
    !std::isfinite(parameters.fine_angular_resolution) ||
    !std::isfinite(parameters.translation_penalty_weight) ||
    !std::isfinite(parameters.rotation_penalty_weight) ||
    !std::isfinite(parameters.minimum_score) ||
    parameters.grid_resolution <= 0.0 ||
    parameters.smear_deviation <= 0.0 ||
    parameters.linear_search_window < 0.0 ||
    parameters.angular_search_window < 0.0 ||
    parameters.coarse_linear_resolution <= 0.0 ||
    parameters.coarse_angular_resolution <= 0.0 ||
    parameters.fine_linear_resolution <= 0.0 ||
    parameters.fine_angular_resolution <= 0.0 ||
    parameters.translation_penalty_weight < 0.0 ||
    parameters.rotation_penalty_weight < 0.0 ||
    parameters.minimum_score < 0.0 ||
    parameters.minimum_score > 1.0 ||
    parameters.minimum_matched_points == 0U)
  {
    throw std::invalid_argument(
            "Invalid correlative scan matcher parameters");
  }
}

CorrelativeScanMatcherResult matchCorrelative(
  const std::vector<Point2D> & reference_points,
  const std::vector<Point2D> & current_points,
  const Pose2D & predicted_pose,
  const CorrelativeScanMatcherParameters & parameters)
{
  validateCorrelativeScanMatcherParameters(parameters);
  if (!isFinitePose(predicted_pose) ||
    std::any_of(
      reference_points.begin(), reference_points.end(),
      [](const Point2D & point) {
        return !std::isfinite(point.x) || !std::isfinite(point.y);
      }) ||
    std::any_of(
      current_points.begin(), current_points.end(),
      [](const Point2D & point) {
        return !std::isfinite(point.x) || !std::isfinite(point.y);
      }))
  {
    throw std::invalid_argument(
            "Correlative scan matcher inputs must be finite");
  }
  CorrelativeScanMatcherResult result;
  result.pose = predicted_pose;
  if (reference_points.empty() ||
    current_points.size() < parameters.minimum_matched_points)
  {
    return result;
  }

  const CorrelationGrid grid(
    reference_points,
    parameters.grid_resolution,
    parameters.smear_deviation);
  CandidateScore best = searchWindow(
    grid,
    current_points,
    predicted_pose,
    predicted_pose,
    parameters.linear_search_window,
    parameters.angular_search_window,
    parameters.coarse_linear_resolution,
    parameters.coarse_angular_resolution,
    parameters,
    result.evaluated_candidates);
  best = searchWindow(
    grid,
    current_points,
    best.pose,
    predicted_pose,
    parameters.coarse_linear_resolution,
    parameters.coarse_angular_resolution,
    parameters.fine_linear_resolution,
    parameters.fine_angular_resolution,
    parameters,
    result.evaluated_candidates);

  result.pose = best.pose;
  result.score = best.score;
  result.matched_points = best.matched_points;
  result.success =
    best.score >= parameters.minimum_score &&
    best.matched_points >= parameters.minimum_matched_points;
  return result;
}

}  // namespace slam_robot_slam
