#ifndef SLAM_ROBOT_SLAM__CORRELATIVE_SCAN_MATCHER_HPP_
#define SLAM_ROBOT_SLAM__CORRELATIVE_SCAN_MATCHER_HPP_

#include <cstddef>
#include <vector>

#include "slam_robot_slam/pose2d.hpp"

namespace slam_robot_slam
{

struct CorrelativeScanMatcherParameters
{
  double grid_resolution{0.05};
  double smear_deviation{0.10};
  double linear_search_window{0.15};
  double angular_search_window{0.20};
  double coarse_linear_resolution{0.04};
  double coarse_angular_resolution{0.04};
  double fine_linear_window{0.02};
  double fine_angular_window{0.02};
  double fine_linear_resolution{0.005};
  double fine_angular_resolution{0.005};
  double translation_penalty_weight{0.10};
  double rotation_penalty_weight{0.10};
  bool degeneracy_handling_enabled{false};
  double minimum_translation_information{1.0};
  double minimum_translation_information_ratio{0.05};
  double weak_direction_correction_scale{0.0};
  double minimum_score{0.35};
  double minimum_support_fraction{0.25};
  std::size_t minimum_matched_points{40U};
};

struct CorrelativeScanMatcherResult
{
  Pose2D pose;
  bool success{false};
  double score{0.0};
  std::size_t matched_points{0U};
  std::size_t supported_points{0U};
  double support_fraction{0.0};
  std::size_t evaluated_candidates{0U};
  std::size_t translation_observable_rank{2U};
  double minimum_translation_information{0.0};
  double maximum_translation_information{0.0};
  double translation_information_ratio{1.0};
  Point2D weak_translation_direction{1.0F, 0.0F};
};

void validateCorrelativeScanMatcherParameters(
  const CorrelativeScanMatcherParameters & parameters);

CorrelativeScanMatcherResult matchCorrelative(
  const std::vector<Point2D> & reference_points,
  const std::vector<Point2D> & current_points,
  const Pose2D & predicted_pose,
  const CorrelativeScanMatcherParameters & parameters);

}  // namespace slam_robot_slam

#endif  // SLAM_ROBOT_SLAM__CORRELATIVE_SCAN_MATCHER_HPP_
