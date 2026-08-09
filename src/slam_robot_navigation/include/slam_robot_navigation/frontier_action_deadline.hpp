// Copyright 2026 Jerry

#ifndef SLAM_ROBOT_NAVIGATION__FRONTIER_ACTION_DEADLINE_HPP_
#define SLAM_ROBOT_NAVIGATION__FRONTIER_ACTION_DEADLINE_HPP_

#include <chrono>
#include <optional>

namespace slam_robot_navigation
{

// Small ROS-independent watchdog used for path requests, navigation and
// cancellation.  Wall time is intentional: a stalled simulation clock must
// not prevent an action client from recovering.
class FrontierActionDeadline
{
public:
  using Clock = std::chrono::steady_clock;

  void arm(Clock::time_point now, Clock::duration timeout)
  {
    deadline_ = now + timeout;
  }

  void disarm()
  {
    deadline_.reset();
  }

  bool armed() const
  {
    return deadline_.has_value();
  }

  bool expired(Clock::time_point now) const
  {
    return deadline_.has_value() && now > *deadline_;
  }

private:
  std::optional<Clock::time_point> deadline_;
};

}  // namespace slam_robot_navigation

#endif  // SLAM_ROBOT_NAVIGATION__FRONTIER_ACTION_DEADLINE_HPP_
