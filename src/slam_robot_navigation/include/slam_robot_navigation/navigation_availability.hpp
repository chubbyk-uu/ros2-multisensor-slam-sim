// Copyright 2026 Jerry

#ifndef SLAM_ROBOT_NAVIGATION__NAVIGATION_AVAILABILITY_HPP_
#define SLAM_ROBOT_NAVIGATION__NAVIGATION_AVAILABILITY_HPP_

#include <cstddef>

#include "slam_robot_navigation/frontier_action_deadline.hpp"

namespace slam_robot_navigation
{

// A closed vocabulary, because the regression classifies runs by it. Free text
// would put the verdict at the mercy of a reworded sentence, and the direction
// of that error is bad: an unmatched failure falls back to a core failure and
// condemns the algorithm for the host's behaviour.
constexpr const char * kFailureClassDependencyLost = "dependency_lost";
constexpr const char * kFailureClassInternal = "internal";
constexpr const char * kFailureCodeNav2StartupTimeout = "nav2_startup_timeout";
constexpr const char * kFailureCodeNav2RuntimeLost = "nav2_runtime_lost";
constexpr const char * kFailureCodeInternalState = "internal_state_error";

// What a goal response means once it arrives.
//
// Kept as a decision of its own because one of the four cases is easy to write
// wrongly and impossible to see in a healthy run: a response for a request the
// explorer stopped waiting for, carrying a handle the server did accept. The
// obvious guard returns early on staleness and drops that handle, which leaves
// the robot driving to a frontier nobody is tracking and no reference left to
// cancel it by. A stale acceptance has to be cancelled, not ignored.
enum class GoalResponse
{
  kAccepted,
  kRejected,
  kStaleAccepted,
  kStaleRejected,
};

inline GoalResponse classifyGoalResponse(bool accepted, bool still_wanted)
{
  if (still_wanted) {
    return accepted ? GoalResponse::kAccepted : GoalResponse::kRejected;
  }
  return accepted ? GoalResponse::kStaleAccepted : GoalResponse::kStaleRejected;
}

// Decides when a missing Nav2 stops being a wait and becomes a fault.
//
// Discovery is not readiness. Nav2 creates its action servers in on_configure
// and only starts accepting goals in on_activate, so a server that is visible
// on the graph rejects every goal for the whole length of that window. Treating
// the first rejection as fatal turns that window into a lost run; treating it
// as a reason to blacklist the frontier blames a perfectly good boundary for an
// infrastructure state. So a rejection is only ever a retry, and giving up is a
// question of how long the condition has lasted.
//
// The budget is steady-clock: the dependency whose loss this is built to catch
// is also the one that publishes /clock, so a simulation-time budget would stop
// advancing at exactly the moment it needed to expire.
class NavigationAvailability
{
public:
  using Clock = FrontierActionDeadline::Clock;

  enum class Status
  {
    kWaiting,       // Discovered or not, no goal accepted yet; budget running.
    kOperational,   // NavigateToPose took a goal, and nothing has failed since.
    kLost,          // The budget expired. Terminal.
  };

  // Call once per cycle, and only once the explorer genuinely needs Nav2: it
  // has a map, a robot pose and a reachable frontier. Called any earlier, the
  // budget would bill SLAM's map initialisation to Nav2's startup.
  Status observe(bool servers_discovered, Clock::time_point now, Clock::duration budget)
  {
    if (status_ == Status::kLost) {return status_;}
    servers_discovered_ = servers_discovered;
    if (servers_discovered && !unusable_since_accepted_) {
      budget_.disarm();
      status_ = navigation_goal_accepted_once_ ? Status::kOperational : Status::kWaiting;
      return status_;
    }
    if (!budget_.armed()) {
      budget_.arm(now, budget);
    } else if (budget_.expired(now)) {
      status_ = Status::kLost;
      return status_;
    }
    status_ = Status::kWaiting;
    return status_;
  }

  // Only NavigateToPose acceptance proves the chain is operational. The planner
  // activates independently of bt_navigator, so a ComputePathToPose that was
  // taken says nothing about whether a navigation goal would be.
  void observeNavigationGoalAccepted()
  {
    navigation_goal_accepted_once_ = true;
    unusable_since_accepted_ = false;
    budget_.disarm();
  }

  void observePlannerGoalAccepted()
  {
    planner_goal_accepted_once_ = true;
  }

  // Two different observations of the same condition: the server did not take
  // the goal. They share the budget, because a chain that answers "no" and one
  // that does not answer are equally unable to explore, but they are counted
  // apart -- reading a log afterwards, "rejected" and "never replied" point at
  // different things to go and look at.
  void observeGoalRejected()
  {
    ++rejected_goals_;
    unusable_since_accepted_ = true;
  }

  void observeGoalUnanswered()
  {
    ++unanswered_goals_;
    unusable_since_accepted_ = true;
  }

  Status status() const {return status_;}
  bool serversDiscovered() const {return servers_discovered_;}
  bool everOperational() const {return navigation_goal_accepted_once_;}
  bool plannerEverAccepted() const {return planner_goal_accepted_once_;}
  std::size_t rejectedGoals() const {return rejected_goals_;}
  std::size_t unansweredGoals() const {return unanswered_goals_;}

  // Which of the two ways this went wrong. "Was a navigation goal ever taken"
  // is the only honest discriminator: a run whose servers appeared but never
  // accepted anything never started, however long it was visible.
  const char * failureCode() const
  {
    return navigation_goal_accepted_once_ ?
           kFailureCodeNav2RuntimeLost : kFailureCodeNav2StartupTimeout;
  }

private:
  Status status_{Status::kWaiting};
  FrontierActionDeadline budget_;
  bool servers_discovered_{false};
  bool navigation_goal_accepted_once_{false};
  bool planner_goal_accepted_once_{false};
  bool unusable_since_accepted_{false};
  std::size_t rejected_goals_{0};
  std::size_t unanswered_goals_{0};
};

}  // namespace slam_robot_navigation

#endif  // SLAM_ROBOT_NAVIGATION__NAVIGATION_AVAILABILITY_HPP_
