// Copyright 2026 Jerry

#ifndef SLAM_ROBOT_NAVIGATION__EMPTY_FRONTIER_COUNTER_HPP_
#define SLAM_ROBOT_NAVIGATION__EMPTY_FRONTIER_COUNTER_HPP_

#include <chrono>
#include <cstddef>

namespace slam_robot_navigation
{

// Counts how many independent looks at the world agreed that no frontier
// remains.
//
// Polling iterations are not independent. An explorer polling faster than the
// projection republishes can reach its threshold without the map ever
// changing, so "nothing left to explore" would really mean "the snapshot I
// already had did not show anything" -- which is also true in the moments
// right after the robot consumed the last frontier that snapshot contained,
// before the mapper published the region beyond it.
//
// A new map revision is therefore what makes a look independent. But requiring
// only that would deadlock exactly when exploration really is finished: with
// no frontier to drive to the robot stops, and a stopped robot produces no
// keyframes and so no new map. A map that has stayed unchanged for longer than
// a republish would take is itself the second kind of independent evidence --
// no update is coming, so the snapshot is the final word rather than one about
// to be superseded.
class EmptyFrontierCounter
{
public:
  using Clock = std::chrono::steady_clock;

  explicit EmptyFrontierCounter(Clock::duration stale_interval)
  : stale_interval_(stale_interval) {}

  void observeEmpty(std::size_t map_revision, Clock::time_point now)
  {
    if (!has_look_ || map_revision != last_revision_ ||
      now - last_look_ >= stale_interval_)
    {
      last_revision_ = map_revision;
      last_look_ = now;
      has_look_ = true;
      ++count_;
    }
  }

  void observeFrontier()
  {
    count_ = 0U;
    has_look_ = false;
  }

  std::size_t count() const
  {
    return count_;
  }

private:
  Clock::duration stale_interval_;
  std::size_t count_{0U};
  std::size_t last_revision_{0U};
  Clock::time_point last_look_;
  bool has_look_{false};
};

}  // namespace slam_robot_navigation

#endif  // SLAM_ROBOT_NAVIGATION__EMPTY_FRONTIER_COUNTER_HPP_
