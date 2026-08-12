// Copyright 2026 Jerry

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "slam_robot_navigation/empty_frontier_counter.hpp"
#include "slam_robot_navigation/frontier_action_deadline.hpp"
#include "slam_robot_navigation/frontier_detector.hpp"
#include "slam_robot_navigation/frontier_goal_selector.hpp"
#include "slam_robot_navigation/navigation_availability.hpp"

namespace slam_robot_navigation
{
namespace
{

double pathLength(const nav_msgs::msg::Path & path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    length += std::hypot(
      path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
      path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
  }
  return length;
}

diagnostic_msgs::msg::KeyValue keyValue(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = std::move(key);
  result.value = std::move(value);
  return result;
}

}  // namespace

class FrontierExplorerNode : public rclcpp::Node
{
public:
  using ComputePath = nav2_msgs::action::ComputePathToPose;
  using Navigate = nav2_msgs::action::NavigateToPose;
  using ComputeGoalHandle = rclcpp_action::ClientGoalHandle<ComputePath>;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<Navigate>;

  FrontierExplorerNode()
  : Node("frontier_explorer"), detector_(declareDetectorParameters())
  {
    const int evaluation_candidate_limit =
      declare_parameter<int>("evaluation_candidate_limit", 5);
    path_cost_weight_ = declare_parameter<double>("path_cost_weight", 1.0);
    const double selection_top_score_band_fraction =
      declare_parameter<double>("selection_top_score_band_fraction", 0.20);
    const std::int64_t selection_random_seed =
      declare_parameter<std::int64_t>("selection_random_seed", 0);
    blacklist_radius_ = declare_parameter<double>("blacklist_radius", 0.5);
    blacklist_duration_ = declare_parameter<double>("blacklist_duration", 60.0);
    navigation_timeout_ = declare_parameter<double>("navigation_timeout", 120.0);
    action_response_timeout_ =
      declare_parameter<double>("action_response_timeout", 15.0);
    // How long Nav2 may stay unable to take a goal before the run is called a
    // dependency failure. Startup is generous because a lifecycle activation
    // on a loaded host is a legitimate wait and its rejections are expected;
    // runtime is short because a chain that was already carrying goals and
    // stopped is a fault, and absorbing it silently wastes the whole budget.
    nav2_startup_grace_ = declare_parameter<double>("nav2_startup_grace", 30.0);
    nav2_runtime_grace_ = declare_parameter<double>("nav2_runtime_grace", 5.0);
    const int minimum_known_free_cells =
      declare_parameter<int>("minimum_known_free_cells", 100);
    const int completion_empty_cycles =
      declare_parameter<int>("completion_empty_cycles", 5);
    const int minimum_goal_free_cell_growth =
      declare_parameter<int>("minimum_goal_free_cell_growth", 100);
    const int maximum_stagnant_goals =
      declare_parameter<int>("maximum_stagnant_goals", 3);
    completion_map_stale_timeout_ =
      declare_parameter<double>("completion_map_stale_timeout", 10.0);
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    save_snapshot_on_completion_ =
      declare_parameter<bool>("save_snapshot_on_completion", true);
    snapshot_service_ = declare_parameter<std::string>(
      "snapshot_service", "/scan_to_map_odometry_3d/save_snapshot");
    const double update_rate = declare_parameter<double>("update_rate", 1.0);
    if (evaluation_candidate_limit <= 0 || minimum_known_free_cells <= 0 ||
      completion_empty_cycles <= 0 || minimum_goal_free_cell_growth <= 0 ||
      maximum_stagnant_goals <= 0 || !std::isfinite(path_cost_weight_) ||
      path_cost_weight_ < 0.0 || !std::isfinite(blacklist_radius_) || blacklist_radius_ <= 0.0 ||
      !std::isfinite(selection_top_score_band_fraction) ||
      selection_top_score_band_fraction < 0.0 || selection_top_score_band_fraction > 1.0 ||
      selection_random_seed < 0 ||
      !std::isfinite(blacklist_duration_) || blacklist_duration_ <= 0.0 ||
      !std::isfinite(navigation_timeout_) || navigation_timeout_ <= 0.0 ||
      !std::isfinite(action_response_timeout_) || action_response_timeout_ <= 0.0 ||
      !std::isfinite(nav2_startup_grace_) || nav2_startup_grace_ <= 0.0 ||
      !std::isfinite(nav2_runtime_grace_) || nav2_runtime_grace_ <= 0.0 ||
      !std::isfinite(completion_map_stale_timeout_) ||
      completion_map_stale_timeout_ <= 0.0 ||
      map_topic_.empty() || (save_snapshot_on_completion_ && snapshot_service_.empty()) ||
      !std::isfinite(update_rate) || update_rate <= 0.0)
    {
      throw std::invalid_argument("invalid frontier explorer parameters");
    }
    evaluation_candidate_limit_ = static_cast<std::size_t>(evaluation_candidate_limit);
    goal_selector_ = std::make_unique<FrontierGoalSelector>(
      selection_top_score_band_fraction,
      static_cast<std::uint64_t>(selection_random_seed));
    minimum_known_free_cells_ = static_cast<std::size_t>(minimum_known_free_cells);
    completion_empty_cycles_ = static_cast<std::size_t>(completion_empty_cycles);
    minimum_goal_free_cell_growth_ = static_cast<std::size_t>(minimum_goal_free_cell_growth);
    maximum_stagnant_goals_ = static_cast<std::size_t>(maximum_stagnant_goals);
    empty_frontiers_ = EmptyFrontierCounter(
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(completion_map_stale_timeout_)));

    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) {
        latest_map_ = std::move(map);
        ++map_revision_;
      });
    completion_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "~/complete", rclcpp::QoS(1).reliable().transient_local());
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "~/diagnostics", 10);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/markers", rclcpp::QoS(1).reliable().transient_local());
    status_marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/status_marker", rclcpp::QoS(1).reliable().transient_local());
    if (save_snapshot_on_completion_) {
      snapshot_client_ = create_client<std_srvs::srv::Trigger>(snapshot_service_);
    }
    compute_path_client_ = rclcpp_action::create_client<ComputePath>(this, "/compute_path_to_pose");
    navigate_client_ = rclcpp_action::create_client<Navigate>(this, "/navigate_to_pose");
    shutdown_callback_handle_ = get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      std::bind(&FrontierExplorerNode::stopActionCallbacks, this));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate),
      std::bind(&FrontierExplorerNode::tick, this));
    publishCompletion(false);
    RCLCPP_INFO(
      get_logger(), "Frontier near-best goal selection seed: %" PRIu64,
      goal_selector_->effectiveSeed());
  }

private:
  enum class State {kWaiting, kPlanning, kNavigating, kCanceling, kComplete, kFault};

  struct BlacklistedGoal
  {
    double x{0.0};
    double y{0.0};
    std::chrono::steady_clock::time_point expires;
  };

  FrontierDetectorParameters declareDetectorParameters()
  {
    FrontierDetectorParameters parameters;
    const int free_maximum = declare_parameter<int>("frontier.free_maximum", 20);
    const int occupied_minimum = declare_parameter<int>("frontier.occupied_minimum", 65);
    const int minimum_cluster_cells = declare_parameter<int>("frontier.minimum_cluster_cells", 8);
    if (free_maximum < 0 || free_maximum > 100 || occupied_minimum < 0 ||
      occupied_minimum > 100 || minimum_cluster_cells <= 0)
    {
      throw std::invalid_argument("invalid integer frontier detector parameters");
    }
    parameters.free_maximum = static_cast<std::int8_t>(free_maximum);
    parameters.occupied_minimum = static_cast<std::int8_t>(occupied_minimum);
    parameters.minimum_cluster_cells = static_cast<std::size_t>(minimum_cluster_cells);
    parameters.minimum_clearance = declare_parameter<double>("frontier.minimum_clearance", 0.25);
    parameters.information_gain_weight =
      declare_parameter<double>("frontier.information_gain_weight", 2.0);
    parameters.distance_weight = declare_parameter<double>("frontier.distance_weight", 1.0);
    parameters.clearance_weight = declare_parameter<double>("frontier.clearance_weight", 0.5);
    return parameters;
  }

  void stopActionCallbacks()
  {
    std::lock_guard<std::mutex> lock(action_handle_mutex_);
    if (compute_goal_handle_) {compute_path_client_->stop_callbacks(compute_goal_handle_);}
    if (navigate_goal_handle_) {navigate_client_->stop_callbacks(navigate_goal_handle_);}
  }

  std::optional<std::pair<double, double>> robotPosition()
  {
    try {
      const auto transform = tf_buffer_->lookupTransform("map", "base_footprint",
          tf2::TimePointZero);
      return std::pair<double, double>{
        transform.transform.translation.x, transform.transform.translation.y};
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for map -> base_footprint: %s", error.what());
      return std::nullopt;
    }
  }

  bool blacklisted(const FrontierCandidate & candidate) const
  {
    return std::any_of(blacklist_.begin(), blacklist_.end(), [&](const auto & entry) {
               return std::hypot(candidate.x - entry.x, candidate.y - entry.y) <= blacklist_radius_;
      });
  }

  bool targetStillTraversable() const
  {
    if (!latest_map_ || !current_target_) {return false;}
    const auto & map = *latest_map_;
    const double resolution = map.info.resolution;
    const int x = static_cast<int>(std::floor(
        (current_target_->x - map.info.origin.position.x) / resolution));
    const int y = static_cast<int>(std::floor(
        (current_target_->y - map.info.origin.position.y) / resolution));
    if (x < 0 || y < 0 || x >= static_cast<int>(map.info.width) ||
      y >= static_cast<int>(map.info.height))
    {
      return false;
    }
    const auto value = map.data[static_cast<std::size_t>(y) * map.info.width + x];
    // Reaching a frontier normally turns its surrounding unknown cells into
    // known space before the robot arrives.  That is progress, not a reason
    // to churn the action.  Replan only when the target itself is no longer a
    // known traversable cell; Nav2 remains responsible for path blockage.
    return value >= 0 && value <= detector_.freeMaximum();
  }

  std::chrono::steady_clock::duration actionResponseTimeout() const
  {
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(action_response_timeout_));
  }

  void finishCanceledNavigation()
  {
    if (cancel_with_blacklist_) {
      blacklistCurrentTarget();
    }
    if (cancel_counts_as_failure_) {
      ++failed_goals_;
    }
    current_target_.reset();
    state_ = State::kWaiting;
    navigation_deadline_.disarm();
    cancellation_deadline_.disarm();
    ++navigation_request_id_;
  }

  // Callers log the cause themselves: a navigation timeout is a fault, while
  // replanning a goal the growing map turned untraversable is routine.
  void cancelNavigation(bool blacklist_target, bool count_as_failure)
  {
    cancel_with_blacklist_ = blacklist_target;
    cancel_counts_as_failure_ = count_as_failure;
    state_ = State::kCanceling;
    cancellation_deadline_.arm(
      std::chrono::steady_clock::now(), actionResponseTimeout());
    const auto goal_handle = navigateGoalHandle();
    if (goal_handle) {
      navigate_client_->async_cancel_goal(goal_handle);
    } else {
      finishCanceledNavigation();
    }
  }

  void tick()
  {
    const auto now = std::chrono::steady_clock::now();
    blacklist_.erase(
      std::remove_if(blacklist_.begin(), blacklist_.end(),
      [now](const auto & entry) {return entry.expires <= now;}),
      blacklist_.end());
    if (!latest_map_ || state_ == State::kComplete || state_ == State::kFault) {
      publishDiagnostics();
      return;
    }
    // Only once the explorer has actually asked Nav2 for something. Before
    // that there is nothing to be unavailable for, and a budget started at the
    // first map would charge SLAM's initialisation to Nav2.
    if (navigation_needed_ &&
      navigation_availability_.observe(
        navigationServersDiscovered(), now, navigationGrace()) ==
      NavigationAvailability::Status::kLost)
    {
      abortExploration(
        kFailureClassDependencyLost, navigation_availability_.failureCode(),
        navigation_availability_.everOperational() ?
        "Nav2 stopped accepting goals after the run had started" :
        "Nav2 never accepted a navigation goal");
      return;
    }
    if (state_ == State::kNavigating) {
      if (navigation_deadline_.expired(now)) {
        RCLCPP_WARN(get_logger(), "Frontier navigation timed out; canceling goal");
        cancelNavigation(true, true);
        return;
      }
    }
    if (state_ == State::kCanceling) {
      if (cancellation_deadline_.expired(now)) {
        RCLCPP_ERROR(get_logger(), "Frontier navigation cancel request timed out");
        finishCanceledNavigation();
      }
      publishDiagnostics();
      return;
    }
    if (state_ == State::kPlanning) {
      if (active_planning_candidate_ && planning_deadline_.expired(now)) {
        RCLCPP_WARN(get_logger(), "Frontier path request timed out; skipping candidate");
        blacklistCandidate(*active_planning_candidate_);
        ++unreachable_candidates_;
        active_planning_candidate_.reset();
        planning_deadline_.disarm();
        ++planning_request_id_;
        path_request_pending_ = true;
      }
      if (path_request_pending_) {
        path_request_pending_ = false;
        requestNextPath();
      }
      publishDiagnostics();
      return;
    }
    const auto robot = robotPosition();
    if (!robot) {
      publishDiagnostics();
      return;
    }
    if (!running_marker_published_) {
      publishStatusMarker("EXPLORATION\nRUNNING", 1.0F, 0.8F, 0.1F);
      running_marker_published_ = true;
    }
    if (state_ == State::kNavigating) {
      if (map_revision_ > target_map_revision_ && !targetStillTraversable()) {
        RCLCPP_INFO(
          get_logger(), "Frontier goal became untraversable; canceling and replanning");
        // The target is no longer free in the newest map. Blacklist its
        // neighbourhood so a frontier shifted by one or two cells cannot be
        // selected again and consume another full navigation timeout.
        cancelNavigation(true, false);
      }
      publishDiagnostics();
      return;
    }
    if (stagnant_goals_ >= maximum_stagnant_goals_ &&
      knownFreeCells() >= minimum_known_free_cells_)
    {
      completeExploration("known free area stopped growing");
      return;
    }

    auto detected = detector_.detect(*latest_map_, robot->first, robot->second);
    raw_frontier_candidates_ = detected.size();
    detected.erase(
      std::remove_if(detected.begin(), detected.end(),
      [this](const auto & candidate) {return blacklisted(candidate);}),
      detected.end());
    blacklisted_frontier_candidates_ = raw_frontier_candidates_ - detected.size();
    publishMarkers(detected);
    if (detected.empty()) {
      empty_frontiers_.observeEmpty(map_revision_, now);
      if (empty_frontiers_.count() >= completion_empty_cycles_ &&
        knownFreeCells() >= minimum_known_free_cells_)
      {
        completeExploration("no reachable frontier remains");
      }
      publishDiagnostics();
      return;
    }
    empty_frontiers_.observeFrontier();
    // Map, robot pose and a reachable frontier: this is the first moment the
    // explorer genuinely needs Nav2, so this is where its budget starts.
    navigation_needed_ = true;
    if (!navigationServersDiscovered()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for Nav2 action servers");
      publishDiagnostics();
      return;
    }
    if (detected.size() > evaluation_candidate_limit_) {
      detected.resize(evaluation_candidate_limit_);
    }
    planning_candidates_ = std::move(detected);
    planning_index_ = 0;
    scored_candidates_.clear();
    state_ = State::kPlanning;
    path_request_pending_ = true;
    active_planning_candidate_.reset();
    planning_deadline_.disarm();
  }

  geometry_msgs::msg::PoseStamped candidatePose(const FrontierCandidate & candidate) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    // A zero stamp asks Nav2 to use the latest available TF.  Stamping a goal
    // with the current simulation time can put it one odometry sample ahead of
    // map -> odom and cause avoidable future-extrapolation failures.
    pose.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    pose.pose.position.x = candidate.x;
    pose.pose.position.y = candidate.y;
    pose.pose.orientation.z = std::sin(candidate.yaw * 0.5);
    pose.pose.orientation.w = std::cos(candidate.yaw * 0.5);
    return pose;
  }

  void requestNextPath()
  {
    if (planning_index_ >= planning_candidates_.size()) {
      const auto selection = goal_selector_->select(scored_candidates_);
      if (selection) {
        last_selection_rank_ = selection->rank;
        last_selection_pool_size_ = selection->pool_size;
        last_selection_score_ = selection->score;
        last_selection_candidate_count_ = scored_candidates_.size();
        last_selection_scores_ = formatCandidateScores(scored_candidates_);
        ++selection_decisions_;
        // Written per candidate so the whole decision survives in the run log:
        // which rule was applied is a parameter choice, and replaying another
        // one offline needs every candidate the selector actually saw, not
        // only the one that won.
        RCLCPP_INFO_STREAM(
          get_logger(),
          formatSelectionRecord(
            selection_decisions_, scored_candidates_.size(), selection->pool_size,
            selection->rank, goal_selector_->effectiveSeed(), selection->score));
        for (std::size_t index = 0U; index < scored_candidates_.size(); ++index) {
          RCLCPP_INFO_STREAM(
            get_logger(),
            formatCandidateRecord(
              selection_decisions_, index, scored_candidates_[index],
              index == selection->index));
        }
        startNavigation(selection->candidate);
      } else {
        state_ = State::kWaiting;
      }
      return;
    }
    const FrontierCandidate candidate = planning_candidates_[planning_index_++];
    const std::size_t request_id = ++planning_request_id_;
    active_planning_candidate_ = candidate;
    planning_deadline_.arm(
      std::chrono::steady_clock::now(), actionResponseTimeout());
    ComputePath::Goal goal;
    goal.goal = candidatePose(candidate);
    goal.use_start = false;
    auto options = rclcpp_action::Client<ComputePath>::SendGoalOptions();
    options.goal_response_callback = [this,
        candidate, request_id](const ComputeGoalHandle::SharedPtr & handle) {
        if (state_ != State::kPlanning || request_id != planning_request_id_) {
          return;
        }
        if (!handle) {
          // A rejection is a statement about the server, not about this
          // frontier: Nav2 rejects while it is configured but not yet active.
          // Blacklisting here would retire a good boundary for an
          // infrastructure state, so the candidate is put back and retried
          // until the availability budget runs out.
          active_planning_candidate_.reset();
          planning_deadline_.disarm();
          if (planning_index_ > 0U) {--planning_index_;}
          navigation_availability_.observeGoalRejected();
          path_request_pending_ = true;
        } else {
          navigation_availability_.observePlannerGoalAccepted();
          std::lock_guard<std::mutex> lock(action_handle_mutex_);
          compute_goal_handle_ = handle;
        }
      };
    options.result_callback = [this, candidate,
        request_id](const ComputeGoalHandle::WrappedResult & result) {
        if (state_ != State::kPlanning || request_id != planning_request_id_) {
          return;
        }
        {
          std::lock_guard<std::mutex> lock(action_handle_mutex_);
          compute_goal_handle_.reset();
        }
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
          result.result->error_code == ComputePath::Result::NONE &&
          result.result->path.poses.size() >= 2)
        {
          const double length = pathLength(result.result->path);
          const double score = candidate.score - path_cost_weight_ * length;
          scored_candidates_.push_back({candidate, score, length});
        } else {
          ++unreachable_candidates_;
          blacklistCandidate(candidate);
        }
        active_planning_candidate_.reset();
        planning_deadline_.disarm();
        path_request_pending_ = true;
      };
    compute_path_client_->async_send_goal(goal, options);
  }

  void startNavigation(const FrontierCandidate & candidate)
  {
    Navigate::Goal goal;
    goal.pose = candidatePose(candidate);
    current_target_ = candidate;
    goal_start_free_cells_ = knownFreeCells();
    target_map_revision_ = map_revision_;
    auto options = rclcpp_action::Client<Navigate>::SendGoalOptions();
    const std::size_t request_id = ++navigation_request_id_;
    options.goal_response_callback = [this,
        request_id](const NavigateGoalHandle::SharedPtr & handle) {
        if (state_ != State::kNavigating || request_id != navigation_request_id_) {
          return;
        }
        {
          std::lock_guard<std::mutex> lock(action_handle_mutex_);
          navigate_goal_handle_ = handle;
        }
        if (!handle) {
          // Same reasoning as the path request: not a failed goal, not a
          // blacklisted target, just a server that is not accepting yet. Back
          // to kWaiting so the next cycle re-detects and tries again.
          navigation_availability_.observeGoalRejected();
          state_ = State::kWaiting;
        } else {
          // The only proof the whole chain is operational. The planner
          // activates independently of bt_navigator, so a taken path request
          // says nothing about whether a navigation goal would be taken.
          navigation_availability_.observeNavigationGoalAccepted();
        }
      };
    options.result_callback = [this, request_id](const NavigateGoalHandle::WrappedResult & result) {
        if ((state_ != State::kNavigating && state_ != State::kCanceling) ||
          request_id != navigation_request_id_)
        {
          return;
        }
        {
          std::lock_guard<std::mutex> lock(action_handle_mutex_);
          navigate_goal_handle_.reset();
        }
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          ++successful_goals_;
          if (map_revision_ == target_map_revision_) {
            // The map never republished while this goal ran, so both free-cell
            // counts come from the same message and the measured growth is
            // zero by construction. That says nothing about whether the robot
            // explored, so it must not count toward stagnation either way.
            ++unmeasured_goal_growth_;
          } else {
            const std::size_t current_free_cells = knownFreeCells();
            const std::size_t growth = current_free_cells > goal_start_free_cells_ ?
              current_free_cells - goal_start_free_cells_ : 0U;
            if (growth < minimum_goal_free_cell_growth_) {
              ++stagnant_goals_;
            } else {
              stagnant_goals_ = 0;
            }
          }
        } else {
          if (result.code == rclcpp_action::ResultCode::CANCELED) {
            if (cancel_with_blacklist_) {
              blacklistCurrentTarget();
            }
            if (cancel_counts_as_failure_) {
              ++failed_goals_;
            }
          } else {
            ++failed_goals_;
            blacklistCurrentTarget();
          }
        }
        current_target_.reset();
        state_ = State::kWaiting;
        navigation_deadline_.disarm();
        cancellation_deadline_.disarm();
      };
    state_ = State::kNavigating;
    cancel_with_blacklist_ = false;
    cancel_counts_as_failure_ = false;
    navigation_deadline_.arm(
      std::chrono::steady_clock::now(),
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(navigation_timeout_)));
    navigate_client_->async_send_goal(goal, options);
    RCLCPP_INFO(
      get_logger(), "Navigating to frontier (%.2f, %.2f), score %.2f",
      candidate.x, candidate.y, last_selection_score_);
  }

  void blacklistCurrentTarget()
  {
    if (!current_target_) {return;}
    blacklistCandidate(*current_target_);
  }

  void blacklistCandidate(const FrontierCandidate & candidate)
  {
    blacklist_.push_back({
        candidate.x, candidate.y,
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(blacklist_duration_))});
  }

  NavigateGoalHandle::SharedPtr navigateGoalHandle()
  {
    std::lock_guard<std::mutex> lock(action_handle_mutex_);
    return navigate_goal_handle_;
  }

  std::size_t knownFreeCells() const
  {
    if (!latest_map_) {return 0;}
    return static_cast<std::size_t>(std::count_if(
        latest_map_->data.begin(), latest_map_->data.end(),
             [this](std::int8_t value) {
               return value >= 0 && value <= detector_.freeMaximum();
             }));
  }

  void publishCompletion(bool complete)
  {
    std_msgs::msg::Bool message;
    message.data = complete;
    completion_publisher_->publish(message);
  }

  void completeExploration(const char * reason)
  {
    state_ = State::kComplete;
    // Published as a diagnostic key, not only logged: a regression that only
    // sees complete=true cannot tell "the map is finished" from "every
    // remaining frontier is temporarily blacklisted".
    completion_reason_ = reason;
    publishCompletion(true);
    const char * next_step = save_snapshot_on_completion_ ?
      "Saving the final snapshot now..." : "It is now safe to press Ctrl+C.";
    const char * marker_next_step = save_snapshot_on_completion_ ?
      "SAVING SNAPSHOT..." : "PRESS CTRL+C TO EXIT";
    publishStatusMarker(
      "EXPLORATION\nCOMPLETE\n" + std::string(marker_next_step), 0.1F, 1.0F, 0.1F);
    RCLCPP_INFO(
      get_logger(),
      "\n============================================================\n"
      "EXPLORATION COMPLETE\n"
      "Reason: %s\n"
      "Goals: %zu succeeded, %zu failed\n"
      "%s\n"
      "============================================================",
      reason, successful_goals_, failed_goals_, next_step);
    publishDiagnostics();
    requestSnapshotSave();
  }

  bool navigationServersDiscovered() const
  {
    return compute_path_client_->action_server_is_ready() &&
           navigate_client_->action_server_is_ready();
  }

  // Two budgets, because the two situations have different natural lengths.
  // Waiting for a lifecycle activation on a loaded host is a startup cost and
  // deserves room; losing a chain that was already carrying goals is a fault
  // and should be called quickly rather than absorbed for half a minute.
  std::chrono::steady_clock::duration navigationGrace() const
  {
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(
        navigation_availability_.everOperational() ?
        nav2_runtime_grace_ : nav2_startup_grace_));
  }

  void abortExploration(
    const char * failure_class, const char * failure_code, const char * reason)
  {
    if (state_ == State::kFault) {return;}
    state_ = State::kFault;
    failure_class_ = failure_class;
    failure_code_ = failure_code;
    failure_reason_ = reason;
    NavigateGoalHandle::SharedPtr navigate_handle;
    ComputeGoalHandle::SharedPtr compute_handle;
    {
      std::lock_guard<std::mutex> lock(action_handle_mutex_);
      navigate_handle = navigate_goal_handle_;
      compute_handle = compute_goal_handle_;
    }
    if (navigate_handle) {
      navigate_client_->async_cancel_goal(navigate_handle);
    }
    if (compute_handle) {
      compute_path_client_->async_cancel_goal(compute_handle);
    }
    ++navigation_request_id_;
    ++planning_request_id_;
    navigation_deadline_.disarm();
    planning_deadline_.disarm();
    cancellation_deadline_.disarm();
    // Deliberately no completion message. Publishing false here would say
    // "exploration is running", which is what the regression reads it as, so
    // the abort would extend the run it is meant to end. The fault travels on
    // the diagnostics instead, where it carries its own classification.
    publishStatusMarker(
      "EXPLORATION\nABORTED\nCHECK THE TERMINAL", 1.0F, 0.1F, 0.1F);
    RCLCPP_ERROR(
      get_logger(),
      "\n============================================================\n"
      "EXPLORATION ABORTED\n"
      "Class: %s  Code: %s\n"
      "Reason: %s\n"
      "No completion message was published and no snapshot was saved.\n"
      "============================================================",
      failure_class, failure_code, reason);
    publishDiagnostics();
  }

  void requestSnapshotSave()
  {
    if (!save_snapshot_on_completion_) {return;}
    if (!snapshot_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "Exploration finished but snapshot service is unavailable");
      publishStatusMarker(
        "EXPLORATION\nCOMPLETE\nSNAPSHOT SAVE FAILED\nCHECK THE TERMINAL",
        1.0F, 0.1F, 0.1F);
      return;
    }
    snapshot_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (response->success) {
            RCLCPP_INFO(
              get_logger(),
              "\n============================================================\n"
              "FINAL EXPLORATION SNAPSHOT SAVED\n"
              "Path: %s\n"
              "It is now safe to inspect the map and press Ctrl+C.\n"
              "============================================================",
              response->message.c_str());
            publishStatusMarker(
              "EXPLORATION\nCOMPLETE\nSNAPSHOT SAVED\nPRESS CTRL+C TO EXIT",
              0.1F, 1.0F, 0.1F);
          } else {
            RCLCPP_ERROR(
              get_logger(), "Final exploration snapshot save failed: %s",
              response->message.c_str());
            publishStatusMarker(
              "EXPLORATION\nCOMPLETE\nSNAPSHOT SAVE FAILED\nCHECK THE TERMINAL",
              1.0F, 0.1F, 0.1F);
          }
        } catch (const std::exception & error) {
          RCLCPP_ERROR(
            get_logger(), "Final exploration snapshot request failed: %s", error.what());
          publishStatusMarker(
            "EXPLORATION\nCOMPLETE\nSNAPSHOT SAVE FAILED\nCHECK THE TERMINAL",
            1.0F, 0.1F, 0.1F);
        }
      });
  }

  void publishDiagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "frontier_explorer";
    status.hardware_id = "navigation";
    status.level = state_ == State::kFault ?
      diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = state_ == State::kComplete ? "complete" :
      (state_ == State::kFault ? "fault" : "running");
    status.values = {
      keyValue("state", std::to_string(static_cast<int>(state_))),
      keyValue("map_revision", std::to_string(map_revision_)),
      keyValue("known_free_cells", std::to_string(knownFreeCells())),
      keyValue("successful_goals", std::to_string(successful_goals_)),
      keyValue("failed_goals", std::to_string(failed_goals_)),
      keyValue("stagnant_goals", std::to_string(stagnant_goals_)),
      keyValue("unreachable_candidates", std::to_string(unreachable_candidates_)),
      keyValue("blacklisted_goals", std::to_string(blacklist_.size())),
      keyValue("raw_frontier_candidates", std::to_string(raw_frontier_candidates_)),
      keyValue(
        "blacklisted_frontier_candidates",
        std::to_string(blacklisted_frontier_candidates_)),
      keyValue("empty_frontier_looks", std::to_string(empty_frontiers_.count())),
      keyValue("unmeasured_goal_growth", std::to_string(unmeasured_goal_growth_)),
      keyValue(
        "selection_random_seed", std::to_string(goal_selector_->effectiveSeed())),
      keyValue("last_selection_rank", std::to_string(last_selection_rank_)),
      keyValue("last_selection_pool_size", std::to_string(last_selection_pool_size_)),
      keyValue(
        "last_selection_candidate_count",
        std::to_string(last_selection_candidate_count_)),
      keyValue("last_selection_scores", last_selection_scores_),
      keyValue("selection_decisions", std::to_string(selection_decisions_)),
      keyValue("completion_reason", completion_reason_),
      // Classification first, prose second. The regression reads only the two
      // closed-vocabulary fields; failure_reason stays for people.
      keyValue("failure_class", failure_class_),
      keyValue("failure_code", failure_code_),
      keyValue("failure_reason", failure_reason_),
      keyValue(
        "nav2_rejected_goals",
        std::to_string(navigation_availability_.rejectedGoals())),
      keyValue(
        "nav2_operational", navigation_availability_.everOperational() ? "true" : "false")};
    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(array);
  }

  void publishMarkers(const std::vector<FrontierCandidate> & candidates)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    int id = 0;
    for (const auto & candidate : candidates) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = now();
      marker.ns = "frontier_candidates";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = candidate.x;
      marker.pose.position.y = candidate.y;
      marker.pose.position.z = 0.05;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.12;
      marker.color.r = 0.1F;
      marker.color.g = 1.0F;
      marker.color.b = 0.2F;
      marker.color.a = 0.9F;
      array.markers.push_back(std::move(marker));
    }
    marker_publisher_->publish(array);
  }

  void publishStatusMarker(const std::string & text, float red, float green, float blue)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_footprint";
    marker.header.stamp = now();
    marker.ns = "exploration_status";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.z = 2.2;
    marker.pose.orientation.w = 1.0;
    marker.frame_locked = true;
    marker.scale.z = 0.14;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0F;
    marker.text = text;
    status_marker_publisher_->publish(marker);
  }

  FrontierDetector detector_;
  std::unique_ptr<FrontierGoalSelector> goal_selector_;
  std::size_t evaluation_candidate_limit_{5};
  double path_cost_weight_{1.0};
  double blacklist_radius_{0.5};
  double blacklist_duration_{60.0};
  double navigation_timeout_{120.0};
  double action_response_timeout_{15.0};
  std::size_t minimum_known_free_cells_{100};
  std::size_t completion_empty_cycles_{5};
  std::size_t minimum_goal_free_cell_growth_{100};
  std::size_t maximum_stagnant_goals_{3};
  std::string map_topic_{"/map"};
  std::string snapshot_service_{"/scan_to_map_odometry_3d/save_snapshot"};
  std::string completion_reason_;
  std::string failure_class_;
  std::string failure_code_;
  std::string failure_reason_;
  double nav2_startup_grace_{30.0};
  double nav2_runtime_grace_{5.0};
  bool navigation_needed_{false};
  bool save_snapshot_on_completion_{true};
  bool running_marker_published_{false};
  State state_{State::kWaiting};
  NavigationAvailability navigation_availability_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map_;
  std::size_t map_revision_{0};
  std::size_t target_map_revision_{0};
  double completion_map_stale_timeout_{10.0};
  EmptyFrontierCounter empty_frontiers_{std::chrono::seconds(10)};
  std::size_t unmeasured_goal_growth_{0};
  std::size_t raw_frontier_candidates_{0U};
  std::size_t blacklisted_frontier_candidates_{0U};
  std::size_t successful_goals_{0};
  std::size_t failed_goals_{0};
  std::size_t stagnant_goals_{0};
  std::size_t goal_start_free_cells_{0};
  std::size_t unreachable_candidates_{0};
  std::vector<FrontierCandidate> planning_candidates_;
  std::vector<ScoredFrontierCandidate> scored_candidates_;
  std::size_t planning_index_{0};
  bool path_request_pending_{false};
  std::size_t last_selection_rank_{0U};
  std::size_t last_selection_pool_size_{0U};
  std::size_t last_selection_candidate_count_{0U};
  std::size_t selection_decisions_{0U};
  std::string last_selection_scores_;
  double last_selection_score_{0.0};
  bool cancel_with_blacklist_{false};
  bool cancel_counts_as_failure_{false};
  FrontierActionDeadline navigation_deadline_;
  FrontierActionDeadline planning_deadline_;
  FrontierActionDeadline cancellation_deadline_;
  std::size_t planning_request_id_{0U};
  std::size_t navigation_request_id_{0U};
  std::optional<FrontierCandidate> active_planning_candidate_;
  std::optional<FrontierCandidate> current_target_;
  std::vector<BlacklistedGoal> blacklist_;
  NavigateGoalHandle::SharedPtr navigate_goal_handle_;
  ComputeGoalHandle::SharedPtr compute_goal_handle_;
  std::mutex action_handle_mutex_;
  rclcpp::PreShutdownCallbackHandle shutdown_callback_handle_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completion_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr status_marker_publisher_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr snapshot_client_;
  rclcpp_action::Client<ComputePath>::SharedPtr compute_path_client_;
  rclcpp_action::Client<Navigate>::SharedPtr navigate_client_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace slam_robot_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<slam_robot_navigation::FrontierExplorerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("frontier_explorer"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
