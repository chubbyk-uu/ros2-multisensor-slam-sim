#ifndef SLAM_ROBOT_SLAM_3D__MATCH_FAILURE_RECOVERY_HPP_
#define SLAM_ROBOT_SLAM_3D__MATCH_FAILURE_RECOVERY_HPP_

#include <cstddef>

#include "slam_robot_slam_3d/scan_to_map_matcher.hpp"

namespace slam_robot_slam_3d
{

class MatchFailureRecovery
{
public:
  explicit MatchFailureRecovery(std::size_t maximum_consecutive_failures);

  bool observe(ScanToMapStatus status);
  std::size_t consecutiveFailures() const;
  std::size_t reinitializationCount() const;

private:
  std::size_t maximum_consecutive_failures_;
  std::size_t consecutive_failures_{0U};
  std::size_t reinitialization_count_{0U};
};

}  // namespace slam_robot_slam_3d

#endif  // SLAM_ROBOT_SLAM_3D__MATCH_FAILURE_RECOVERY_HPP_
