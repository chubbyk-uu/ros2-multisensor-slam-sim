// Copyright 2026 Jerry

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
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

#include "slam_robot_navigation/frontier_detector.hpp"

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
    blacklist_radius_ = declare_parameter<double>("blacklist_radius", 0.5);
    blacklist_duration_ = declare_parameter<double>("blacklist_duration", 60.0);
    navigation_timeout_ = declare_parameter<double>("navigation_timeout", 120.0);
    const int minimum_known_free_cells =
      declare_parameter<int>("minimum_known_free_cells", 100);
    const int completion_empty_cycles =
      declare_parameter<int>("completion_empty_cycles", 5);
    const int minimum_goal_free_cell_growth =
      declare_parameter<int>("minimum_goal_free_cell_growth", 100);
    const int maximum_stagnant_goals =
      declare_parameter<int>("maximum_stagnant_goals", 3);
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
      !std::isfinite(blacklist_duration_) || blacklist_duration_ <= 0.0 ||
      !std::isfinite(navigation_timeout_) || navigation_timeout_ <= 0.0 ||
      map_topic_.empty() || (save_snapshot_on_completion_ && snapshot_service_.empty()) ||
      !std::isfinite(update_rate) || update_rate <= 0.0)
    {
      throw std::invalid_argument("invalid frontier explorer parameters");
    }
    evaluation_candidate_limit_ = static_cast<std::size_t>(evaluation_candidate_limit);
    minimum_known_free_cells_ = static_cast<std::size_t>(minimum_known_free_cells);
    completion_empty_cycles_ = static_cast<std::size_t>(completion_empty_cycles);
    minimum_goal_free_cell_growth_ = static_cast<std::size_t>(minimum_goal_free_cell_growth);
    maximum_stagnant_goals_ = static_cast<std::size_t>(maximum_stagnant_goals);

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
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", 10);
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
  }

private:
  enum class State {kWaiting, kPlanning, kNavigating, kCanceling, kComplete};

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
    return value >= 0 && value <= 20;
  }

  void tick()
  {
    const auto now = std::chrono::steady_clock::now();
    blacklist_.erase(
      std::remove_if(blacklist_.begin(), blacklist_.end(),
      [now](const auto & entry) {return entry.expires <= now;}),
      blacklist_.end());
    if (!latest_map_ || state_ == State::kComplete || state_ == State::kCanceling) {
      publishDiagnostics();
      return;
    }
    const auto robot = robotPosition();
    if (!robot) {return;}
    if (state_ == State::kNavigating) {
      if (now > navigation_deadline_) {
        RCLCPP_WARN(get_logger(), "Frontier navigation timed out; canceling goal");
        cancel_with_blacklist_ = true;
        state_ = State::kCanceling;
        const auto goal_handle = navigateGoalHandle();
        if (goal_handle) {
          navigate_client_->async_cancel_goal(goal_handle);
        } else {
          blacklistCurrentTarget();
          ++failed_goals_;
          current_target_.reset();
          state_ = State::kWaiting;
        }
        return;
      }
      if (map_revision_ > target_map_revision_ && !targetStillTraversable()) {
        RCLCPP_INFO(get_logger(), "Frontier goal became untraversable; canceling and replanning");
        state_ = State::kCanceling;
        const auto goal_handle = navigateGoalHandle();
        if (goal_handle) {
          navigate_client_->async_cancel_goal(goal_handle);
        } else {
          state_ = State::kWaiting;
        }
      }
      publishDiagnostics();
      return;
    }
    if (state_ == State::kPlanning) {
      if (path_request_pending_) {
        path_request_pending_ = false;
        requestNextPath();
      }
      return;
    }
    if (stagnant_goals_ >= maximum_stagnant_goals_ &&
      knownFreeCells() >= minimum_known_free_cells_)
    {
      completeExploration("known free area stopped growing");
      return;
    }

    auto detected = detector_.detect(*latest_map_, robot->first, robot->second);
    detected.erase(
      std::remove_if(detected.begin(), detected.end(),
      [this](const auto & candidate) {return blacklisted(candidate);}),
      detected.end());
    publishMarkers(detected);
    if (detected.empty()) {
      ++empty_cycles_;
      if (empty_cycles_ >= completion_empty_cycles_ &&
        knownFreeCells() >= minimum_known_free_cells_)
      {
        completeExploration("no reachable frontier remains");
      }
      publishDiagnostics();
      return;
    }
    empty_cycles_ = 0;
    if (!compute_path_client_->action_server_is_ready() ||
      !navigate_client_->action_server_is_ready())
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for Nav2 action servers");
      return;
    }
    if (detected.size() > evaluation_candidate_limit_) {
      detected.resize(evaluation_candidate_limit_);
    }
    planning_candidates_ = std::move(detected);
    planning_index_ = 0;
    best_candidate_.reset();
    best_path_score_ = -std::numeric_limits<double>::infinity();
    state_ = State::kPlanning;
    path_request_pending_ = true;
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
      if (best_candidate_) {startNavigation(*best_candidate_);} else {state_ = State::kWaiting;}
      return;
    }
    const FrontierCandidate candidate = planning_candidates_[planning_index_++];
    ComputePath::Goal goal;
    goal.goal = candidatePose(candidate);
    goal.use_start = false;
    auto options = rclcpp_action::Client<ComputePath>::SendGoalOptions();
    options.goal_response_callback = [this,
        candidate](const ComputeGoalHandle::SharedPtr & handle) {
        if (!handle) {
          ++unreachable_candidates_;
          blacklistCandidate(candidate);
          path_request_pending_ = true;
        } else {
          std::lock_guard<std::mutex> lock(action_handle_mutex_);
          compute_goal_handle_ = handle;
        }
      };
    options.result_callback = [this, candidate](const ComputeGoalHandle::WrappedResult & result) {
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
          if (score > best_path_score_) {
            best_path_score_ = score;
            best_candidate_ = candidate;
          }
        } else {
          ++unreachable_candidates_;
          blacklistCandidate(candidate);
        }
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
    options.goal_response_callback = [this](const NavigateGoalHandle::SharedPtr & handle) {
        {
          std::lock_guard<std::mutex> lock(action_handle_mutex_);
          navigate_goal_handle_ = handle;
        }
        if (!handle) {
          blacklistCurrentTarget();
          ++failed_goals_;
          state_ = State::kWaiting;
        }
      };
    options.result_callback = [this](const NavigateGoalHandle::WrappedResult & result) {
        {
          std::lock_guard<std::mutex> lock(action_handle_mutex_);
          navigate_goal_handle_.reset();
        }
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          ++successful_goals_;
          const std::size_t current_free_cells = knownFreeCells();
          const std::size_t growth = current_free_cells > goal_start_free_cells_ ?
            current_free_cells - goal_start_free_cells_ : 0U;
          if (growth < minimum_goal_free_cell_growth_) {
            ++stagnant_goals_;
          } else {
            stagnant_goals_ = 0;
          }
        } else {
          if (result.code != rclcpp_action::ResultCode::CANCELED || cancel_with_blacklist_) {
            ++failed_goals_;
            blacklistCurrentTarget();
          }
        }
        current_target_.reset();
        state_ = State::kWaiting;
      };
    state_ = State::kNavigating;
    cancel_with_blacklist_ = false;
    navigation_deadline_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(navigation_timeout_));
    navigate_client_->async_send_goal(goal, options);
    RCLCPP_INFO(
      get_logger(), "Navigating to frontier (%.2f, %.2f), score %.2f",
      candidate.x, candidate.y, best_path_score_);
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
             [](std::int8_t value) {return value >= 0 && value <= 20;}));
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
    requestSnapshotSave();
    publishCompletion(true);
    RCLCPP_INFO(
      get_logger(), "Exploration complete (%s) after %zu successful goals and %zu failed goals",
      reason, successful_goals_, failed_goals_);
    publishDiagnostics();
  }

  void requestSnapshotSave()
  {
    if (!save_snapshot_on_completion_) {return;}
    if (!snapshot_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "Exploration finished but snapshot service is unavailable");
      return;
    }
    snapshot_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (response->success) {
            RCLCPP_INFO(
              get_logger(), "Final exploration snapshot saved to %s", response->message.c_str());
          } else {
            RCLCPP_ERROR(
              get_logger(), "Final exploration snapshot save failed: %s",
              response->message.c_str());
          }
        } catch (const std::exception & error) {
          RCLCPP_ERROR(
            get_logger(), "Final exploration snapshot request failed: %s", error.what());
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
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = state_ == State::kComplete ? "complete" : "running";
    status.values = {
      keyValue("state", std::to_string(static_cast<int>(state_))),
      keyValue("map_revision", std::to_string(map_revision_)),
      keyValue("known_free_cells", std::to_string(knownFreeCells())),
      keyValue("successful_goals", std::to_string(successful_goals_)),
      keyValue("failed_goals", std::to_string(failed_goals_)),
      keyValue("stagnant_goals", std::to_string(stagnant_goals_)),
      keyValue("unreachable_candidates", std::to_string(unreachable_candidates_)),
      keyValue("blacklisted_goals", std::to_string(blacklist_.size()))};
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

  FrontierDetector detector_;
  std::size_t evaluation_candidate_limit_{5};
  double path_cost_weight_{1.0};
  double blacklist_radius_{0.5};
  double blacklist_duration_{60.0};
  double navigation_timeout_{120.0};
  std::size_t minimum_known_free_cells_{100};
  std::size_t completion_empty_cycles_{5};
  std::size_t minimum_goal_free_cell_growth_{100};
  std::size_t maximum_stagnant_goals_{3};
  std::string map_topic_{"/map"};
  std::string snapshot_service_{"/scan_to_map_odometry_3d/save_snapshot"};
  bool save_snapshot_on_completion_{true};
  State state_{State::kWaiting};
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map_;
  std::size_t map_revision_{0};
  std::size_t target_map_revision_{0};
  std::size_t empty_cycles_{0};
  std::size_t successful_goals_{0};
  std::size_t failed_goals_{0};
  std::size_t stagnant_goals_{0};
  std::size_t goal_start_free_cells_{0};
  std::size_t unreachable_candidates_{0};
  std::vector<FrontierCandidate> planning_candidates_;
  std::size_t planning_index_{0};
  bool path_request_pending_{false};
  std::optional<FrontierCandidate> best_candidate_;
  double best_path_score_{0.0};
  bool cancel_with_blacklist_{false};
  std::chrono::steady_clock::time_point navigation_deadline_;
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
