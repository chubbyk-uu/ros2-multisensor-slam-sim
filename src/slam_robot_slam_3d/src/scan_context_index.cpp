#include "slam_robot_slam_3d/scan_context_index.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <pcl/common/point_tests.h>

namespace slam_robot_slam_3d
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double ringKeyDistance(const Eigen::VectorXf & first, const Eigen::VectorXf & second)
{
  return (first - second).norm();
}

std::pair<double, std::size_t> alignedDescriptorDistance(
  const Eigen::MatrixXf & current, const Eigen::MatrixXf & historical)
{
  double best_distance = std::numeric_limits<double>::infinity();
  std::size_t best_shift = 0U;
  for (Eigen::Index shift = 0; shift < current.cols(); ++shift) {
    double similarity_sum = 0.0;
    std::size_t compared_sectors = 0U;
    for (Eigen::Index column = 0; column < current.cols(); ++column) {
      const Eigen::Index historical_column = (column + shift) % current.cols();
      const auto current_sector = current.col(column);
      const auto historical_sector = historical.col(historical_column);
      const double current_norm = current_sector.norm();
      const double historical_norm = historical_sector.norm();
      if (current_norm == 0.0 && historical_norm == 0.0) {
        continue;
      }
      ++compared_sectors;
      if (current_norm > 0.0 && historical_norm > 0.0) {
        similarity_sum += std::clamp(
          static_cast<double>(current_sector.dot(historical_sector)) /
          (current_norm * historical_norm), 0.0, 1.0);
      }
    }
    const double distance = compared_sectors == 0U ?
      std::numeric_limits<double>::infinity() :
      1.0 - similarity_sum / static_cast<double>(compared_sectors);
    if (distance < best_distance) {
      best_distance = distance;
      best_shift = static_cast<std::size_t>(shift);
    }
  }
  return {best_distance, best_shift};
}

}  // namespace

ScanContextIndex::ScanContextIndex(ScanContextParameters parameters)
: parameters_(std::move(parameters))
{
  validateParameters();
}

std::vector<ScanContextCandidate> ScanContextIndex::addAndQuery(
  const GlobalKeyframe & keyframe)
{
  if (!keyframe.filtered_scan || keyframe.filtered_scan->empty() ||
    !keyframe.front_end_base_pose.matrix().allFinite() ||
    !keyframe.base_to_sensor.matrix().allFinite() ||
    !std::isfinite(keyframe.accumulated_distance))
  {
    throw std::invalid_argument("scan context keyframe is invalid");
  }
  Entry current;
  current.keyframe_id = keyframe.id;
  current.accumulated_distance = keyframe.accumulated_distance;
  current.descriptor = makeDescriptor(keyframe);

  const auto candidates = query(current);
  entries_.push_back(std::move(current));
  return candidates;
}

std::size_t ScanContextIndex::size() const
{
  return entries_.size();
}

const ScanContextQueryDiagnostics & ScanContextIndex::lastQueryDiagnostics() const
{
  return last_query_diagnostics_;
}

ScanContextIndex::Descriptor ScanContextIndex::makeDescriptor(
  const GlobalKeyframe & keyframe) const
{
  Descriptor result;
  result.cells = Eigen::MatrixXf::Zero(
    static_cast<Eigen::Index>(parameters_.radial_bins),
    static_cast<Eigen::Index>(parameters_.angular_bins));
  for (const auto & point : *keyframe.filtered_scan) {
    if (!pcl::isFinite(point)) {
      continue;
    }
    const double radius = std::hypot(point.x, point.y);
    if (radius >= parameters_.maximum_radius) {
      continue;
    }
    const double normalized_angle = std::atan2(point.y, point.x) + kPi;
    const auto radial_bin = static_cast<Eigen::Index>(
      radius / parameters_.maximum_radius * parameters_.radial_bins);
    const auto angular_bin = static_cast<Eigen::Index>(
      normalized_angle / kTwoPi * parameters_.angular_bins);
    const Eigen::Index clamped_radial_bin = std::min(
      radial_bin, result.cells.rows() - 1);
    const Eigen::Index clamped_angular_bin = std::min(
      angular_bin, result.cells.cols() - 1);
    const Eigen::Vector3d point_in_base = keyframe.base_to_sensor *
      Eigen::Vector3d(point.x, point.y, point.z);
    // Scan Context stores height above the robot ground reference. This keeps
    // sensor-frame floor returns near zero while preserving lower walls and
    // obstacles that would otherwise be lost as negative sensor-frame z.
    result.cells(clamped_radial_bin, clamped_angular_bin) = std::max(
      result.cells(clamped_radial_bin, clamped_angular_bin),
      static_cast<float>(point_in_base.z()));
  }
  result.ring_key = result.cells.rowwise().mean();
  return result;
}

std::vector<ScanContextCandidate> ScanContextIndex::query(
  const Entry & current)
{
  last_query_diagnostics_ = {};
  struct RingKeyCandidate
  {
    const Entry * entry;
    double distance;
  };
  std::vector<RingKeyCandidate> ring_key_candidates;
  for (const auto & historical : entries_) {
    if (current.keyframe_id <= historical.keyframe_id ||
      current.keyframe_id - historical.keyframe_id <
      parameters_.minimum_keyframe_separation ||
      current.accumulated_distance - historical.accumulated_distance <
      parameters_.minimum_travel_distance)
    {
      continue;
    }
    ++last_query_diagnostics_.eligible_candidates;
    ring_key_candidates.push_back(
      {&historical, ringKeyDistance(current.descriptor.ring_key,
      historical.descriptor.ring_key)});
  }
  std::sort(
    ring_key_candidates.begin(), ring_key_candidates.end(),
    [](const auto & first, const auto & second) {
      return first.distance < second.distance;
    });
  if (ring_key_candidates.size() > parameters_.ring_key_candidate_count) {
    ring_key_candidates.resize(parameters_.ring_key_candidate_count);
  }
  last_query_diagnostics_.shortlisted_candidates = ring_key_candidates.size();

  std::vector<ScanContextCandidate> candidates;
  candidates.reserve(ring_key_candidates.size());
  for (const auto & ring_key_candidate : ring_key_candidates) {
    const auto [descriptor_distance, shift] = alignedDescriptorDistance(
      current.descriptor.cells, ring_key_candidate.entry->descriptor.cells);
    if (last_query_diagnostics_.best_descriptor_distance < 0.0 ||
      descriptor_distance < last_query_diagnostics_.best_descriptor_distance)
    {
      last_query_diagnostics_.best_descriptor_distance = descriptor_distance;
    }
    if (descriptor_distance <= 0.05) {
      ++last_query_diagnostics_.distance_at_most_0_05;
    }
    if (descriptor_distance <= 0.10) {
      ++last_query_diagnostics_.distance_at_most_0_10;
    }
    if (descriptor_distance <= 0.15) {
      ++last_query_diagnostics_.distance_at_most_0_15;
    }
    if (descriptor_distance > parameters_.maximum_descriptor_distance) {
      ++last_query_diagnostics_.descriptor_rejections;
      continue;
    }
    candidates.push_back({
        ring_key_candidate.entry->keyframe_id,
        descriptor_distance,
        ring_key_candidate.distance,
        -kTwoPi * static_cast<double>(shift) /
        static_cast<double>(parameters_.angular_bins)});
  }
  std::sort(
    candidates.begin(), candidates.end(),
    [](const auto & first, const auto & second) {
      return first.descriptor_distance < second.descriptor_distance;
    });
  if (candidates.size() > parameters_.maximum_candidates) {
    candidates.resize(parameters_.maximum_candidates);
  }
  last_query_diagnostics_.accepted_candidates = candidates.size();
  return candidates;
}

void ScanContextIndex::validateParameters() const
{
  if (!std::isfinite(parameters_.maximum_radius) ||
    parameters_.maximum_radius <= 0.0 || parameters_.radial_bins == 0U ||
    parameters_.angular_bins == 0U ||
    parameters_.minimum_keyframe_separation == 0U ||
    !std::isfinite(parameters_.minimum_travel_distance) ||
    parameters_.minimum_travel_distance < 0.0 ||
    !std::isfinite(parameters_.maximum_descriptor_distance) ||
    parameters_.maximum_descriptor_distance <= 0.0 ||
    parameters_.maximum_descriptor_distance > 1.0 ||
    parameters_.ring_key_candidate_count == 0U ||
    parameters_.maximum_candidates == 0U)
  {
    throw std::invalid_argument("scan context parameters are invalid");
  }
}

}  // namespace slam_robot_slam_3d
