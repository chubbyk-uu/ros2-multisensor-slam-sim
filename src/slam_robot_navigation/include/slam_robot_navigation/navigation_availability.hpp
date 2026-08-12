// Copyright 2026 Jerry

#ifndef SLAM_ROBOT_NAVIGATION__NAVIGATION_AVAILABILITY_HPP_
#define SLAM_ROBOT_NAVIGATION__NAVIGATION_AVAILABILITY_HPP_

namespace slam_robot_navigation
{

class NavigationAvailability
{
public:
  enum class Status {kWaiting, kReady, kLost};

  Status observe(bool planner_ready, bool navigator_ready)
  {
    if (status_ == Status::kLost) {
      return status_;
    }
    if (planner_ready && navigator_ready) {
      status_ = Status::kReady;
    } else if (status_ == Status::kReady) {
      status_ = Status::kLost;
    }
    return status_;
  }

  Status goalRejected()
  {
    status_ = Status::kLost;
    return status_;
  }

  Status status() const {return status_;}

private:
  Status status_{Status::kWaiting};
};

}  // namespace slam_robot_navigation

#endif  // SLAM_ROBOT_NAVIGATION__NAVIGATION_AVAILABILITY_HPP_
