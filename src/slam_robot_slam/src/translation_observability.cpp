#include "slam_robot_slam/translation_observability.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace slam_robot_slam
{
namespace
{

bool isFinitePoint(const ScanPoint2D & point)
{
  return std::isfinite(point.point.x) &&
         std::isfinite(point.point.y) &&
         std::isfinite(point.range) &&
         point.range > 0.0F;
}

double distance(const Point2D & first, const Point2D & second)
{
  return std::hypot(
    static_cast<double>(second.x) - static_cast<double>(first.x),
    static_cast<double>(second.y) - static_cast<double>(first.y));
}

bool belongsToSameSurface(
  const ScanPoint2D & first,
  const ScanPoint2D & second,
  const TranslationObservabilityParameters & parameters)
{
  if (second.beam_index <= first.beam_index ||
    second.beam_index - first.beam_index != parameters.beam_index_step)
  {
    return false;
  }
  const double maximum_distance =
    parameters.maximum_neighbor_distance_base +
    parameters.maximum_neighbor_distance_ratio *
    std::min(
    static_cast<double>(first.range),
    static_cast<double>(second.range));
  return distance(first.point, second.point) <= maximum_distance;
}

}  // namespace

void validateTranslationObservabilityParameters(
  const TranslationObservabilityParameters & parameters)
{
  if (parameters.beam_index_step == 0U ||
    parameters.normal_half_window == 0U ||
    !std::isfinite(parameters.maximum_neighbor_distance_base) ||
    parameters.maximum_neighbor_distance_base < 0.0 ||
    !std::isfinite(parameters.maximum_neighbor_distance_ratio) ||
    parameters.maximum_neighbor_distance_ratio < 0.0 ||
    parameters.minimum_normal_count == 0U ||
    !std::isfinite(parameters.minimum_effective_normal_count) ||
    parameters.minimum_effective_normal_count < 0.0 ||
    !std::isfinite(parameters.minimum_information_ratio) ||
    parameters.minimum_information_ratio < 0.0 ||
    parameters.minimum_information_ratio > 1.0)
  {
    throw std::invalid_argument(
            "Invalid translation observability parameters");
  }
}

TranslationObservability estimateNormalTranslationObservability(
  const std::vector<ScanPoint2D> & scan_points,
  const TranslationObservabilityParameters & parameters)
{
  validateTranslationObservabilityParameters(parameters);
  if (std::any_of(
      scan_points.begin(), scan_points.end(),
      [](const ScanPoint2D & point) {return !isFinitePoint(point);}))
  {
    throw std::invalid_argument(
            "Translation observability scan points must be finite");
  }

  double information_xx = 0.0;
  double information_xy = 0.0;
  double information_yy = 0.0;
  std::size_t normal_count = 0U;
  const std::size_t half_window = parameters.normal_half_window;
  for (std::size_t index = half_window;
    index + half_window < scan_points.size(); ++index)
  {
    bool continuous = true;
    for (std::size_t neighbor = index - half_window;
      neighbor < index + half_window; ++neighbor)
    {
      if (!belongsToSameSurface(
          scan_points[neighbor], scan_points[neighbor + 1U], parameters))
      {
        continuous = false;
        break;
      }
    }
    if (!continuous) {
      continue;
    }

    double mean_x = 0.0;
    double mean_y = 0.0;
    const std::size_t window_size = 2U * half_window + 1U;
    for (std::size_t neighbor = index - half_window;
      neighbor <= index + half_window; ++neighbor)
    {
      mean_x += scan_points[neighbor].point.x;
      mean_y += scan_points[neighbor].point.y;
    }
    mean_x /= static_cast<double>(window_size);
    mean_y /= static_cast<double>(window_size);

    double covariance_xx = 0.0;
    double covariance_xy = 0.0;
    double covariance_yy = 0.0;
    for (std::size_t neighbor = index - half_window;
      neighbor <= index + half_window; ++neighbor)
    {
      const double dx = scan_points[neighbor].point.x - mean_x;
      const double dy = scan_points[neighbor].point.y - mean_y;
      covariance_xx += dx * dx;
      covariance_xy += dx * dy;
      covariance_yy += dy * dy;
    }
    const double covariance_trace = covariance_xx + covariance_yy;
    const double covariance_difference = covariance_xx - covariance_yy;
    const double covariance_discriminant =
      std::hypot(covariance_difference, 2.0 * covariance_xy);
    const double maximum_covariance =
      0.5 * (covariance_trace + covariance_discriminant);
    if (maximum_covariance <= std::numeric_limits<double>::epsilon()) {
      continue;
    }
    const double tangent_angle =
      0.5 * std::atan2(2.0 * covariance_xy, covariance_difference);
    const double normal_x = -std::sin(tangent_angle);
    const double normal_y = std::cos(tangent_angle);
    information_xx += normal_x * normal_x;
    information_xy += normal_x * normal_y;
    information_yy += normal_y * normal_y;
    ++normal_count;
  }

  if (normal_count == 0U) {
    return TranslationObservability{};
  }
  const double trace = information_xx + information_yy;
  const double difference = information_xx - information_yy;
  const double discriminant = std::hypot(difference, 2.0 * information_xy);
  const double maximum_information =
    std::max(0.0, 0.5 * (trace + discriminant));
  const double minimum_information =
    std::max(0.0, 0.5 * (trace - discriminant));
  const double maximum_direction_angle =
    0.5 * std::atan2(2.0 * information_xy, difference);
  const Point2D weak_direction{
    static_cast<float>(-std::sin(maximum_direction_angle)),
    static_cast<float>(std::cos(maximum_direction_angle))};
  const double information_ratio =
    maximum_information > std::numeric_limits<double>::epsilon() ?
    minimum_information / maximum_information : 0.0;

  std::size_t rank = 2U;
  if (normal_count < parameters.minimum_normal_count ||
    maximum_information < parameters.minimum_effective_normal_count)
  {
    rank = 0U;
  } else if (
    minimum_information < parameters.minimum_effective_normal_count ||
    information_ratio < parameters.minimum_information_ratio)
  {
    rank = 1U;
  }
  double weak_direction_correction_scale = 1.0;
  if (rank == 0U) {
    weak_direction_correction_scale = 0.0;
  } else if (parameters.minimum_information_ratio > 0.0) {
    weak_direction_correction_scale = std::clamp(
      information_ratio / parameters.minimum_information_ratio,
      0.0,
      1.0);
  }
  return TranslationObservability{
    rank,
    minimum_information,
    maximum_information,
    information_ratio,
    weak_direction,
    normal_count,
    weak_direction_correction_scale};
}

TranslationObservability rotateTranslationObservability(
  const TranslationObservability & observability,
  const double yaw)
{
  if (!std::isfinite(yaw)) {
    throw std::invalid_argument("Observability rotation must be finite");
  }
  TranslationObservability rotated = observability;
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  rotated.weak_direction = Point2D{
    static_cast<float>(
      cosine * observability.weak_direction.x -
      sine * observability.weak_direction.y),
    static_cast<float>(
      sine * observability.weak_direction.x +
      cosine * observability.weak_direction.y)};
  return rotated;
}

}  // namespace slam_robot_slam
