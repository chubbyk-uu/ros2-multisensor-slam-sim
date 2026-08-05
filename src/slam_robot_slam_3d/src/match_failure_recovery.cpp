#include "slam_robot_slam_3d/match_failure_recovery.hpp"

#include <stdexcept>

namespace slam_robot_slam_3d
{
namespace
{

bool isRecoverableRegistrationFailure(ScanToMapStatus status)
{
  return status == ScanToMapStatus::kNotConverged ||
         status == ScanToMapStatus::kInsufficientCorrespondences ||
         status == ScanToMapStatus::kFitnessTooHigh ||
         status == ScanToMapStatus::kCorrectionTooLarge;
}

}  // namespace

MatchFailureRecovery::MatchFailureRecovery(
  std::size_t maximum_consecutive_failures)
: maximum_consecutive_failures_(maximum_consecutive_failures)
{
  if (maximum_consecutive_failures_ == 0U) {
    throw std::invalid_argument("maximum consecutive failures must be positive");
  }
}

bool MatchFailureRecovery::observe(ScanToMapStatus status)
{
  if (status == ScanToMapStatus::kSuccess) {
    consecutive_failures_ = 0U;
    return false;
  }
  if (!isRecoverableRegistrationFailure(status)) {
    return false;
  }
  ++consecutive_failures_;
  if (consecutive_failures_ < maximum_consecutive_failures_) {
    return false;
  }
  consecutive_failures_ = 0U;
  ++reinitialization_count_;
  return true;
}

std::size_t MatchFailureRecovery::consecutiveFailures() const
{
  return consecutive_failures_;
}

std::size_t MatchFailureRecovery::reinitializationCount() const
{
  return reinitialization_count_;
}

}  // namespace slam_robot_slam_3d
