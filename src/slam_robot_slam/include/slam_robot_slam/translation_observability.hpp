#ifndef SLAM_ROBOT_SLAM__TRANSLATION_OBSERVABILITY_HPP_
#define SLAM_ROBOT_SLAM__TRANSLATION_OBSERVABILITY_HPP_

#include <cstddef>
#include <vector>

#include "slam_robot_slam/point2d.hpp"
#include "slam_robot_slam/scan_point2d.hpp"

namespace slam_robot_slam
{

struct TranslationObservabilityParameters
{
  std::size_t beam_index_step{1U};
  std::size_t normal_half_window{3U};
  double maximum_neighbor_distance_base{0.05};
  double maximum_neighbor_distance_ratio{0.05};
  std::size_t minimum_normal_count{20U};
  double minimum_effective_normal_count{5.0};
  double minimum_information_ratio{0.05};
};

struct TranslationObservability
{
  std::size_t rank{0U};
  double minimum_information{0.0};
  double maximum_information{0.0};
  double information_ratio{0.0};
  Point2D weak_direction{1.0F, 0.0F};
  std::size_t normal_count{0U};
  double weak_direction_correction_scale{0.0};
};

void validateTranslationObservabilityParameters(
  const TranslationObservabilityParameters & parameters);

TranslationObservability estimateNormalTranslationObservability(
  const std::vector<ScanPoint2D> & scan_points,
  const TranslationObservabilityParameters & parameters);

TranslationObservability rotateTranslationObservability(
  const TranslationObservability & observability,
  double yaw);

}  // namespace slam_robot_slam

#endif  // SLAM_ROBOT_SLAM__TRANSLATION_OBSERVABILITY_HPP_
