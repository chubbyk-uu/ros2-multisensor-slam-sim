#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "slam_robot_slam/correlative_scan_matcher.hpp"
#include "slam_robot_slam/laser_scan_preprocessor.hpp"
#include "slam_robot_slam/loop_closure_detector.hpp"
#include "slam_robot_slam/loop_closure_processor.hpp"
#include "slam_robot_slam/occupancy_grid_map.hpp"
#include "slam_robot_slam/pose_graph_2d.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace slam_robot_slam
{
namespace
{

int64_t stampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
         static_cast<int64_t>(stamp.nanosec);
}

bool isFinitePositive(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool isFiniteNonNegative(const double value)
{
  return std::isfinite(value) && value >= 0.0;
}

bool hasValidQuaternion(
  const geometry_msgs::msg::Quaternion & orientation)
{
  if (!std::isfinite(orientation.x) ||
    !std::isfinite(orientation.y) ||
    !std::isfinite(orientation.z) ||
    !std::isfinite(orientation.w))
  {
    return false;
  }
  const double squared_norm =
    orientation.x * orientation.x +
    orientation.y * orientation.y +
    orientation.z * orientation.z +
    orientation.w * orientation.w;
  return squared_norm > 1.0e-12;
}

double quaternionToYaw(const geometry_msgs::msg::Quaternion & orientation)
{
  const double sin_yaw =
    2.0 *
    (orientation.w * orientation.z +
    orientation.x * orientation.y);
  const double cos_yaw =
    1.0 -
    2.0 *
    (orientation.y * orientation.y +
    orientation.z * orientation.z);
  return std::atan2(sin_yaw, cos_yaw);
}

geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.z = std::sin(yaw * 0.5);
  orientation.w = std::cos(yaw * 0.5);
  return orientation;
}

}  // namespace

class ScanMatcherOdometryNode : public rclcpp::Node
{
public:
  ScanMatcherOdometryNode()
  : Node("scan_matcher_odometry")
  {
    const auto scan_topic =
      declare_parameter<std::string>("scan_topic", "/scan");
    const auto odom_topic =
      declare_parameter<std::string>("odom_topic", "/odom");
    const auto laser_odom_topic =
      declare_parameter<std::string>(
      "laser_odom_topic", "/custom_slam/laser_odom");
    const auto laser_path_topic =
      declare_parameter<std::string>(
      "laser_path_topic", "/custom_slam/laser_path");
    const auto pose_graph_path_topic =
      declare_parameter<std::string>(
      "pose_graph_path_topic", "/custom_slam/pose_graph_path");
    const auto aligned_points_topic =
      declare_parameter<std::string>(
      "aligned_points_topic", "/custom_slam/aligned_scan_points");
    const auto map_topic =
      declare_parameter<std::string>(
      "map_topic", "/custom_slam/map");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ =
      declare_parameter<std::string>("base_frame", "base_footprint");
    minimum_range_ = declare_parameter<double>("minimum_range", 0.12);
    maximum_range_ = declare_parameter<double>("maximum_range", 12.0);
    const auto point_stride =
      declare_parameter<int64_t>("point_stride", 2);
    maximum_odom_age_ =
      declare_parameter<double>("maximum_odom_age", 0.10);
    minimum_translation_for_update_ =
      declare_parameter<double>("minimum_translation_for_update", 0.05);
    minimum_rotation_for_update_ =
      declare_parameter<double>("minimum_rotation_for_update", 0.05);
    const auto maximum_local_keyframes =
      declare_parameter<int64_t>("maximum_local_keyframes", 20);
    maximum_path_poses_ =
      declare_parameter<int64_t>("maximum_path_poses", 10000);
    const auto map_ray_stride =
      declare_parameter<int64_t>("map.ray_stride", 2);
    const auto map_padding_cells =
      declare_parameter<int64_t>("map.padding_cells", 2);
    const double map_publish_period =
      declare_parameter<double>("map.publish_period", 0.5);
    const auto map_rebuild_keyframes_per_cycle =
      declare_parameter<int64_t>(
      "map.rebuild_keyframes_per_cycle", 4);
    const double map_rebuild_period =
      declare_parameter<double>("map.rebuild_period", 0.02);
    const double sequential_translation_stddev =
      declare_parameter<double>(
      "pose_graph.sequential_translation_stddev", 0.05);
    const double sequential_rotation_stddev =
      declare_parameter<double>(
      "pose_graph.sequential_rotation_stddev", 0.05);
    loop_closure_enabled_ =
      declare_parameter<bool>("loop_closure.enabled", true);
    const auto minimum_keyframe_separation =
      declare_parameter<int64_t>(
      "loop_closure.minimum_keyframe_separation", 80);
    const auto loop_closure_check_interval =
      declare_parameter<int64_t>(
      "loop_closure.check_interval", 10);
    const auto minimum_loop_closure_interval =
      declare_parameter<int64_t>(
      "loop_closure.minimum_loop_closure_interval", 30);
    loop_candidate_parameters_.search_radius =
      declare_parameter<double>("loop_closure.search_radius", 0.8);
    const auto maximum_loop_candidates =
      declare_parameter<int64_t>(
      "loop_closure.maximum_candidates", 3);
    const auto candidate_submap_half_width =
      declare_parameter<int64_t>(
      "loop_closure.candidate_submap_half_width", 5);
    const double loop_translation_stddev =
      declare_parameter<double>(
      "loop_closure.translation_stddev", 0.05);
    const double loop_rotation_stddev =
      declare_parameter<double>(
      "loop_closure.rotation_stddev", 0.05);
    const auto optimization_maximum_iterations =
      declare_parameter<int64_t>(
      "loop_closure.optimization_maximum_iterations", 50);
    pose_graph_optimization_options_.loop_closure_huber_scale =
      declare_parameter<double>("loop_closure.huber_scale", 1.0);

    map_parameters_.resolution =
      declare_parameter<double>("map.resolution", 0.05);
    map_parameters_.hit_probability =
      declare_parameter<double>("map.hit_probability", 0.70);
    map_parameters_.miss_probability =
      declare_parameter<double>("map.miss_probability", 0.40);
    map_parameters_.minimum_probability =
      declare_parameter<double>("map.minimum_probability", 0.12);
    map_parameters_.maximum_probability =
      declare_parameter<double>("map.maximum_probability", 0.97);

    matcher_parameters_.grid_resolution =
      declare_parameter<double>(
      "matcher.grid_resolution", 0.02);
    matcher_parameters_.smear_deviation =
      declare_parameter<double>("matcher.smear_deviation", 0.10);
    matcher_parameters_.linear_search_window =
      declare_parameter<double>("matcher.linear_search_window", 0.15);
    matcher_parameters_.angular_search_window =
      declare_parameter<double>("matcher.angular_search_window", 0.20);
    matcher_parameters_.coarse_linear_resolution =
      declare_parameter<double>("matcher.coarse_linear_resolution", 0.02);
    matcher_parameters_.coarse_angular_resolution =
      declare_parameter<double>("matcher.coarse_angular_resolution", 0.02);
    matcher_parameters_.fine_linear_resolution =
      declare_parameter<double>("matcher.fine_linear_resolution", 0.005);
    matcher_parameters_.fine_angular_resolution =
      declare_parameter<double>("matcher.fine_angular_resolution", 0.005);
    matcher_parameters_.translation_penalty_weight =
      declare_parameter<double>("matcher.translation_penalty_weight", 0.10);
    matcher_parameters_.rotation_penalty_weight =
      declare_parameter<double>("matcher.rotation_penalty_weight", 0.10);
    matcher_parameters_.minimum_score =
      declare_parameter<double>("matcher.minimum_score", 0.35);
    const auto minimum_matched_points =
      declare_parameter<int64_t>("matcher.minimum_matched_points", 40);

    loop_matcher_parameters_.grid_resolution =
      declare_parameter<double>(
      "loop_closure.matcher.grid_resolution", 0.03);
    loop_matcher_parameters_.smear_deviation =
      declare_parameter<double>(
      "loop_closure.matcher.smear_deviation", 0.10);
    loop_matcher_parameters_.linear_search_window =
      declare_parameter<double>(
      "loop_closure.matcher.linear_search_window", 0.40);
    loop_matcher_parameters_.angular_search_window =
      declare_parameter<double>(
      "loop_closure.matcher.angular_search_window", 0.50);
    loop_matcher_parameters_.coarse_linear_resolution =
      declare_parameter<double>(
      "loop_closure.matcher.coarse_linear_resolution", 0.05);
    loop_matcher_parameters_.coarse_angular_resolution =
      declare_parameter<double>(
      "loop_closure.matcher.coarse_angular_resolution", 0.05);
    loop_matcher_parameters_.fine_linear_resolution =
      declare_parameter<double>(
      "loop_closure.matcher.fine_linear_resolution", 0.01);
    loop_matcher_parameters_.fine_angular_resolution =
      declare_parameter<double>(
      "loop_closure.matcher.fine_angular_resolution", 0.01);
    loop_matcher_parameters_.translation_penalty_weight =
      declare_parameter<double>(
      "loop_closure.matcher.translation_penalty_weight", 0.05);
    loop_matcher_parameters_.rotation_penalty_weight =
      declare_parameter<double>(
      "loop_closure.matcher.rotation_penalty_weight", 0.05);
    loop_matcher_parameters_.minimum_score =
      declare_parameter<double>(
      "loop_closure.matcher.minimum_score", 0.55);
    const auto minimum_loop_matched_points =
      declare_parameter<int64_t>(
      "loop_closure.matcher.minimum_matched_points", 100);

    if (!isFiniteNonNegative(minimum_range_) ||
      !isFinitePositive(maximum_range_) ||
      maximum_range_ <= minimum_range_ ||
      point_stride < 1 || !isFinitePositive(maximum_odom_age_) ||
      !isFiniteNonNegative(minimum_translation_for_update_) ||
      !isFiniteNonNegative(minimum_rotation_for_update_) ||
      maximum_local_keyframes < 1 || maximum_path_poses_ < 1 ||
      minimum_matched_points < 1 || map_ray_stride < 1 ||
      !isFinitePositive(map_publish_period) ||
      map_rebuild_keyframes_per_cycle < 1 ||
      !isFinitePositive(map_rebuild_period) ||
      map_padding_cells < 0 ||
      map_padding_cells > std::numeric_limits<int>::max() ||
      !isFinitePositive(sequential_translation_stddev) ||
      !isFinitePositive(sequential_rotation_stddev) ||
      minimum_keyframe_separation < 1 ||
      loop_closure_check_interval < 1 ||
      minimum_loop_closure_interval < 1 ||
      !isFinitePositive(loop_candidate_parameters_.search_radius) ||
      maximum_loop_candidates < 1 ||
      candidate_submap_half_width < 0 ||
      !isFinitePositive(loop_translation_stddev) ||
      !isFinitePositive(loop_rotation_stddev) ||
      optimization_maximum_iterations < 1 ||
      optimization_maximum_iterations >
      std::numeric_limits<int>::max() ||
      !isFinitePositive(
        pose_graph_optimization_options_.loop_closure_huber_scale) ||
      minimum_loop_matched_points < 1)
    {
      throw std::invalid_argument("Invalid scan matcher node parameters");
    }
    point_stride_ = static_cast<std::size_t>(point_stride);
    maximum_local_keyframes_ =
      static_cast<std::size_t>(maximum_local_keyframes);
    matcher_parameters_.minimum_matched_points =
      static_cast<std::size_t>(minimum_matched_points);
    loop_candidate_parameters_.minimum_keyframe_separation =
      static_cast<std::size_t>(minimum_keyframe_separation);
    loop_candidate_parameters_.maximum_candidates =
      static_cast<std::size_t>(maximum_loop_candidates);
    loop_closure_check_interval_ =
      static_cast<std::size_t>(loop_closure_check_interval);
    minimum_loop_closure_interval_ =
      static_cast<std::size_t>(minimum_loop_closure_interval);
    candidate_submap_half_width_ =
      static_cast<std::size_t>(candidate_submap_half_width);
    loop_matcher_parameters_.minimum_matched_points =
      static_cast<std::size_t>(minimum_loop_matched_points);
    validateCorrelativeScanMatcherParameters(matcher_parameters_);
    validateCorrelativeScanMatcherParameters(loop_matcher_parameters_);
    pose_graph_optimization_options_.maximum_iterations =
      static_cast<int>(optimization_maximum_iterations);
    map_ray_stride_ = static_cast<std::size_t>(map_ray_stride);
    map_rebuild_keyframes_per_cycle_ =
      static_cast<std::size_t>(map_rebuild_keyframes_per_cycle);
    map_parameters_.padding_cells = static_cast<int>(map_padding_cells);
    sequential_translation_weight_ =
      1.0 / sequential_translation_stddev;
    sequential_rotation_weight_ =
      1.0 / sequential_rotation_stddev;
    loop_translation_weight_ = 1.0 / loop_translation_stddev;
    loop_rotation_weight_ = 1.0 / loop_rotation_stddev;
    occupancy_grid_map_ =
      std::make_unique<OccupancyGridMap>(map_parameters_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    transform_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    laser_odometry_publisher_ =
      create_publisher<nav_msgs::msg::Odometry>(
      laser_odom_topic, rclcpp::QoS(10).reliable());
    laser_path_publisher_ =
      create_publisher<nav_msgs::msg::Path>(
      laser_path_topic, rclcpp::QoS(1).reliable());
    pose_graph_path_publisher_ =
      create_publisher<nav_msgs::msg::Path>(
      pose_graph_path_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    aligned_points_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      aligned_points_topic, rclcpp::QoS(10).reliable());
    map_publisher_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>(
      map_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    odometry_subscription_ =
      create_subscription<nav_msgs::msg::Odometry>(
      odom_topic,
      rclcpp::QoS(100).reliable(),
      std::bind(
        &ScanMatcherOdometryNode::odometryCallback,
        this,
        std::placeholders::_1));
    laser_scan_subscription_ =
      create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic,
      rclcpp::SensorDataQoS(),
      std::bind(
        &ScanMatcherOdometryNode::laserScanCallback,
        this,
        std::placeholders::_1));
    map_publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(map_publish_period),
      std::bind(
        &ScanMatcherOdometryNode::publishMapIfDirty,
        this));
    map_rebuild_timer_ = create_wall_timer(
      std::chrono::duration<double>(map_rebuild_period),
      std::bind(
        &ScanMatcherOdometryNode::backgroundMaintenanceCallback,
        this));

    path_.header.frame_id = map_frame_;
    pose_graph_path_.header.frame_id = map_frame_;
    RCLCPP_INFO(
      get_logger(),
      "Correlative scan-to-local-submap matcher: %s + %s -> %s, stride %zu",
      scan_topic.c_str(),
      odom_topic.c_str(),
      laser_odom_topic.c_str(),
      point_stride_);
    RCLCPP_INFO(
      get_logger(),
      "Keyframe occupancy grid: %s in frame %s, resolution %.3f m",
      map_topic.c_str(),
      map_frame_.c_str(),
      map_parameters_.resolution);
    RCLCPP_INFO(
      get_logger(),
      "Loop closure %s: history >= %zu keyframes, radius %.2f m, "
      "check every %zu keyframes",
      loop_closure_enabled_ ? "enabled" : "disabled",
      loop_candidate_parameters_.minimum_keyframe_separation,
      loop_candidate_parameters_.search_radius,
      loop_closure_check_interval_);
  }

private:
  struct OdomSample
  {
    int64_t stamp_nanoseconds;
    Pose2D pose;
  };

  struct Keyframe
  {
    builtin_interfaces::msg::Time stamp;
    Pose2D pose;
    std::shared_ptr<const std::vector<Point2D>> points;
    Point2D sensor_origin;
    struct MapRayObservation
    {
      Point2D endpoint;
      bool endpoint_is_hit;
    };
    std::vector<MapRayObservation> map_rays;
  };

  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odometry)
  {
    const auto & position = odometry->pose.pose.position;
    const auto & orientation = odometry->pose.pose.orientation;
    if (!std::isfinite(position.x) ||
      !std::isfinite(position.y) ||
      !hasValidQuaternion(orientation))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Ignoring odometry with non-finite pose or invalid quaternion");
      return;
    }
    const Pose2D pose{
      position.x,
      position.y,
      quaternionToYaw(orientation)};
    if (!isFinitePose(pose)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Ignoring odometry that produced a non-finite planar pose");
      return;
    }
    odom_samples_.push_back(
      OdomSample{
        stampToNanoseconds(odometry->header.stamp),
        pose});

    constexpr std::size_t kMaximumSamples = 300U;
    while (odom_samples_.size() > kMaximumSamples) {
      odom_samples_.pop_front();
    }
  }

  bool findNearestOdometry(
    const builtin_interfaces::msg::Time & stamp,
    Pose2D & pose) const
  {
    if (odom_samples_.empty()) {
      return false;
    }

    const int64_t target = stampToNanoseconds(stamp);
    const OdomSample * nearest = nullptr;
    int64_t nearest_difference = std::numeric_limits<int64_t>::max();
    for (const auto & sample : odom_samples_) {
      const int64_t difference = std::abs(sample.stamp_nanoseconds - target);
      if (difference < nearest_difference) {
        nearest = &sample;
        nearest_difference = difference;
      }
    }

    const int64_t maximum_difference = static_cast<int64_t>(
      maximum_odom_age_ * 1000000000.0);
    if (nearest == nullptr || nearest_difference > maximum_difference) {
      return false;
    }

    pose = nearest->pose;
    return true;
  }

  bool lookupBaseFromLaser(
    const std::string & laser_frame,
    Pose2D & base_from_laser)
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        base_frame_, laser_frame, tf2::TimePointZero);
      if (!std::isfinite(transform.transform.translation.x) ||
        !std::isfinite(transform.transform.translation.y) ||
        !hasValidQuaternion(transform.transform.rotation))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "Ignoring non-finite %s -> %s transform",
          laser_frame.c_str(),
          base_frame_.c_str());
        return false;
      }
      base_from_laser = Pose2D{
        transform.transform.translation.x,
        transform.transform.translation.y,
        quaternionToYaw(transform.transform.rotation)};
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Waiting for %s -> %s transform: %s",
        laser_frame.c_str(),
        base_frame_.c_str(),
        error.what());
      return false;
    }
  }

  void laserScanCallback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
  {
    applyCompletedLoopClosure();
    try {
      processLaserScan(scan);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Rejected LaserScan after processing error: %s",
        error.what());
    }
  }

  void processLaserScan(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
  {
    if (!hasValidLaserScanMetadata(*scan)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Ignoring LaserScan with invalid angular or range metadata");
      return;
    }

    Pose2D odometry_pose;
    if (!findNearestOdometry(scan->header.stamp, odometry_pose)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "No odometry sample close to the LaserScan timestamp");
      return;
    }

    Pose2D base_from_laser;
    if (!lookupBaseFromLaser(scan->header.frame_id, base_from_laser)) {
      return;
    }

    const auto laser_points = projectLaserScan(
      *scan, minimum_range_, maximum_range_, point_stride_);
    std::vector<Point2D> base_points;
    base_points.reserve(laser_points.size());
    std::transform(
      laser_points.begin(),
      laser_points.end(),
      std::back_inserter(base_points),
      [&base_from_laser](const Point2D & point) {
        return transformPoint(base_from_laser, point);
      });
    if (base_points.size() < matcher_parameters_.minimum_matched_points) {
      return;
    }

    if (!initialized_) {
      estimated_pose_ = odometry_pose;
      last_matched_pose_ = odometry_pose;
      last_matched_odometry_pose_ = odometry_pose;
      addKeyframe(
        scan->header.stamp,
        estimated_pose_,
        base_points,
        *scan,
        base_from_laser);
      initialized_ = true;
      publishEstimate(scan->header.stamp, odometry_pose, base_points);
      return;
    }

    const Pose2D odometry_increment =
      relativePose(last_matched_odometry_pose_, odometry_pose);
    const Pose2D predicted_pose =
      composePoses(last_matched_pose_, odometry_increment);
    if (std::hypot(odometry_increment.x, odometry_increment.y) <
      minimum_translation_for_update_ &&
      std::abs(odometry_increment.yaw) < minimum_rotation_for_update_)
    {
      estimated_pose_ = predicted_pose;
      publishEstimate(scan->header.stamp, odometry_pose, base_points);
      return;
    }

    const auto reference_points = buildLocalReferencePoints();
    const CorrelativeScanMatcherResult result = matchCorrelative(
      reference_points,
      base_points,
      predicted_pose,
      matcher_parameters_);

    if (result.success) {
      estimated_pose_ = result.pose;
      last_matched_pose_ = estimated_pose_;
      last_matched_odometry_pose_ = odometry_pose;
      addKeyframe(
        scan->header.stamp,
        estimated_pose_,
        base_points,
        *scan,
        base_from_laser);
      const Pose2D correction =
        relativePose(predicted_pose, result.pose);
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Correlative match score=%.3f points=%zu candidates=%zu "
        "correction=(%.4f m, %.3f deg)",
        result.score,
        result.matched_points,
        result.evaluated_candidates,
        std::hypot(correction.x, correction.y),
        correction.yaw * 180.0 / 3.14159265358979323846);
    } else {
      estimated_pose_ = predicted_pose;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Correlative match rejected (score=%.3f, points=%zu); "
        "using wheel odometry prediction",
        result.score,
        result.matched_points);
    }

    publishEstimate(scan->header.stamp, odometry_pose, base_points);
  }

  void addKeyframe(
    const builtin_interfaces::msg::Time & stamp,
    const Pose2D & pose,
    const std::vector<Point2D> & points,
    const sensor_msgs::msg::LaserScan & scan,
    const Pose2D & base_from_laser)
  {
    validateKeyframeGraphInvariant();
    Keyframe keyframe{
      stamp,
      pose,
      std::make_shared<const std::vector<Point2D>>(points),
      Point2D{
        static_cast<float>(base_from_laser.x),
        static_cast<float>(base_from_laser.y)},
      buildMapRayObservations(scan, base_from_laser)};

    const std::size_t expected_node_id = keyframes_.size();
    const std::size_t node_id = pose_graph_.addNode(pose);
    if (node_id != expected_node_id) {
      pose_graph_.removeLastNode();
      throw std::logic_error(
              "Pose graph node and keyframe indices diverged");
    }

    std::optional<std::size_t> sequential_constraint_id;
    bool keyframe_inserted = false;
    try {
      keyframes_.push_back(std::move(keyframe));
      keyframe_inserted = true;
      if (node_id > 0U) {
        const Pose2D & previous_pose =
          keyframes_[node_id - 1U].pose;
        sequential_constraint_id = pose_graph_.addConstraint(
          PoseGraphConstraint{
            node_id - 1U,
            node_id,
            relativePose(previous_pose, pose),
            sequential_translation_weight_,
            sequential_rotation_weight_,
            PoseGraphConstraintType::kSequential});
      }
      validateKeyframeGraphInvariant();
    } catch (...) {
      if (sequential_constraint_id.has_value()) {
        pose_graph_.removeConstraint(*sequential_constraint_id);
      }
      if (keyframe_inserted) {
        keyframes_.pop_back();
      }
      pose_graph_.removeLastNode();
      throw;
    }

    geometry_msgs::msg::PoseStamped graph_pose;
    graph_pose.header.stamp = stamp;
    graph_pose.header.frame_id = map_frame_;
    graph_pose.pose.position.x = pose.x;
    graph_pose.pose.position.y = pose.y;
    graph_pose.pose.orientation = yawToQuaternion(pose.yaw);
    pose_graph_path_.header = graph_pose.header;
    pose_graph_path_.poses.push_back(graph_pose);
    pose_graph_path_publisher_->publish(pose_graph_path_);

    integrateKeyframeIntoActiveMap(keyframes_.back());
    startLoopClosure(node_id);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Pose graph contains %zu nodes and %zu constraints",
      pose_graph_.nodes().size(),
      pose_graph_.constraints().size());
  }

  void rebuildPoseGraphPath(
    const builtin_interfaces::msg::Time & latest_stamp)
  {
    validateKeyframeGraphInvariant();
    pose_graph_path_.poses.clear();
    pose_graph_path_.poses.reserve(pose_graph_.nodes().size());
    for (std::size_t index = 0U;
      index < pose_graph_.nodes().size(); ++index)
    {
      geometry_msgs::msg::PoseStamped graph_pose;
      graph_pose.header.stamp = keyframes_[index].stamp;
      graph_pose.header.frame_id = map_frame_;
      graph_pose.pose.position.x = pose_graph_.nodes()[index].pose.x;
      graph_pose.pose.position.y = pose_graph_.nodes()[index].pose.y;
      graph_pose.pose.orientation =
        yawToQuaternion(pose_graph_.nodes()[index].pose.yaw);
      pose_graph_path_.poses.push_back(graph_pose);
    }
    pose_graph_path_.header.stamp = latest_stamp;
    pose_graph_path_.header.frame_id = map_frame_;
    pose_graph_path_publisher_->publish(pose_graph_path_);
  }

  void startLoopClosure(const std::size_t current_id)
  {
    validateKeyframeGraphInvariant();
    if (!loop_closure_enabled_ ||
      loop_closure_future_.has_value() ||
      current_id <
      loop_candidate_parameters_.minimum_keyframe_separation ||
      current_id % loop_closure_check_interval_ != 0U ||
      (last_loop_closure_node_.has_value() &&
      current_id - *last_loop_closure_node_ <
      minimum_loop_closure_interval_))
    {
      return;
    }

    std::vector<LoopClosureKeyframe2D> loop_keyframes;
    loop_keyframes.reserve(keyframes_.size());
    for (const auto & keyframe : keyframes_) {
      loop_keyframes.push_back(
        LoopClosureKeyframe2D{keyframe.pose, keyframe.points});
    }

    LoopClosureProcessorParameters parameters;
    parameters.candidate = loop_candidate_parameters_;
    parameters.candidate_submap_half_width =
      candidate_submap_half_width_;
    parameters.matcher = loop_matcher_parameters_;
    parameters.translation_weight = loop_translation_weight_;
    parameters.rotation_weight = loop_rotation_weight_;
    parameters.optimization = pose_graph_optimization_options_;
    PoseGraph2D graph_snapshot = pose_graph_;
    loop_closure_job_started_ = std::chrono::steady_clock::now();
    try {
      loop_closure_future_.emplace(
        std::async(
          std::launch::async,
          [
            loop_keyframes = std::move(loop_keyframes),
            graph_snapshot = std::move(graph_snapshot),
            current_id,
            parameters
          ]() mutable {
            return processLoopClosure(
              loop_keyframes,
              std::move(graph_snapshot),
              current_id,
              parameters);
          }));
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to start loop closure worker: %s",
        error.what());
    }
  }

  void applyCompletedLoopClosure()
  {
    if (!loop_closure_future_.has_value() ||
      loop_closure_future_->wait_for(std::chrono::seconds(0)) !=
      std::future_status::ready)
    {
      return;
    }

    LoopClosureProcessingResult result;
    try {
      result = loop_closure_future_->get();
    } catch (const std::exception & error) {
      loop_closure_future_.reset();
      RCLCPP_ERROR(
        get_logger(),
        "Loop closure worker failed: %s",
        error.what());
      return;
    }
    loop_closure_future_.reset();
    if (!result.accepted) {
      RCLCPP_DEBUG(
        get_logger(),
        "Rejected %zu loop closure candidates for keyframe %zu",
        result.evaluated_candidates,
        result.current_id);
      return;
    }

    try {
      validateKeyframeGraphInvariant();
      if (result.current_id >= keyframes_.size() ||
        result.optimized_graph.nodes().size() !=
        result.current_id + 1U)
      {
        throw std::logic_error(
                "Loop closure result does not match the live graph prefix");
      }

      const Pose2D old_latest_keyframe_pose = keyframes_.back().pose;
      const Pose2D latest_to_estimate =
        relativePose(old_latest_keyframe_pose, estimated_pose_);
      const Pose2D latest_to_last_match =
        relativePose(old_latest_keyframe_pose, last_matched_pose_);
      const Pose2D old_anchor_pose =
        pose_graph_.nodes()[result.current_id].pose;
      const Pose2D new_anchor_pose =
        result.optimized_graph.nodes()[result.current_id].pose;
      const Pose2D map_correction =
        composePoses(new_anchor_pose, inversePose(old_anchor_pose));

      std::vector<Pose2D> merged_poses;
      merged_poses.reserve(pose_graph_.nodes().size());
      for (std::size_t index = 0U;
        index <= result.current_id; ++index)
      {
        merged_poses.push_back(
          result.optimized_graph.nodes()[index].pose);
      }
      for (std::size_t index = result.current_id + 1U;
        index < pose_graph_.nodes().size(); ++index)
      {
        merged_poses.push_back(
          composePoses(map_correction, pose_graph_.nodes()[index].pose));
      }

      PoseGraph2D merged_graph = pose_graph_;
      merged_graph.addConstraint(result.constraint);
      merged_graph.setNodePoses(merged_poses);
      pose_graph_ = std::move(merged_graph);
      for (std::size_t index = 0U; index < keyframes_.size(); ++index) {
        keyframes_[index].pose = merged_poses[index];
      }
      estimated_pose_ =
        composePoses(keyframes_.back().pose, latest_to_estimate);
      last_matched_pose_ =
        composePoses(keyframes_.back().pose, latest_to_last_match);
      last_loop_closure_node_ = result.current_id;
      rebuildPoseGraphPath(keyframes_.back().stamp);
      startMapRebuild();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(),
        "Discarded incompatible loop closure result: %s",
        error.what());
      return;
    }

    const double elapsed_milliseconds =
      std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() -
      loop_closure_job_started_).count();
    RCLCPP_INFO(
      get_logger(),
      "Accepted loop closure %zu -> %zu: score=%.3f points=%zu, "
      "Ceres cost %.6f -> %.6f in %d iterations (worker %.1f ms)",
      result.candidate_id,
      result.current_id,
      result.match.score,
      result.match.matched_points,
      result.optimization.initial_cost,
      result.optimization.final_cost,
      result.optimization.iterations,
      elapsed_milliseconds);
  }

  std::vector<Point2D> buildLocalReferencePoints() const
  {
    validateKeyframeGraphInvariant();
    std::size_t total_points = 0U;
    const std::size_t first_keyframe =
      keyframes_.size() > maximum_local_keyframes_ ?
      keyframes_.size() - maximum_local_keyframes_ : 0U;
    for (std::size_t index = first_keyframe;
      index < keyframes_.size(); ++index)
    {
      total_points += keyframes_[index].points->size();
    }

    std::vector<Point2D> reference_points;
    reference_points.reserve(total_points);
    for (std::size_t index = first_keyframe;
      index < keyframes_.size(); ++index)
    {
      const auto & keyframe = keyframes_[index];
      std::transform(
        keyframe.points->begin(),
        keyframe.points->end(),
        std::back_inserter(reference_points),
        [&keyframe](const Point2D & point) {
          return transformPoint(keyframe.pose, point);
        });
    }
    return reference_points;
  }

  void validateKeyframeGraphInvariant() const
  {
    if (keyframes_.size() != pose_graph_.nodes().size()) {
      throw std::logic_error(
              "Keyframe and pose graph node counts diverged");
    }
    for (std::size_t index = 0U; index < pose_graph_.nodes().size(); ++index) {
      if (pose_graph_.nodes()[index].id != index) {
        throw std::logic_error(
                "Pose graph node IDs are not contiguous");
      }
    }
  }

  std::vector<Keyframe::MapRayObservation> buildMapRayObservations(
    const sensor_msgs::msg::LaserScan & scan,
    const Pose2D & base_from_laser) const
  {
    std::vector<Keyframe::MapRayObservation> observations;
    const double effective_minimum =
      std::max(minimum_range_, static_cast<double>(scan.range_min));
    const double effective_maximum =
      std::min(maximum_range_, static_cast<double>(scan.range_max));
    if (effective_minimum >= effective_maximum) {
      return observations;
    }

    observations.reserve(
      (scan.ranges.size() + map_ray_stride_ - 1U) / map_ray_stride_);
    for (std::size_t index = 0U;
      index < scan.ranges.size();
      index += map_ray_stride_)
    {
      const double measured_range =
        static_cast<double>(scan.ranges[index]);
      double ray_length = effective_maximum;
      bool endpoint_is_hit = false;
      if (std::isfinite(measured_range)) {
        if (measured_range < effective_minimum) {
          continue;
        }
        ray_length = std::min(measured_range, effective_maximum);
        endpoint_is_hit = measured_range < effective_maximum;
      } else if (!std::isinf(measured_range) || measured_range < 0.0) {
        continue;
      }

      const double angle =
        static_cast<double>(scan.angle_min) +
        static_cast<double>(index) *
        static_cast<double>(scan.angle_increment);
      const Point2D laser_endpoint{
        static_cast<float>(ray_length * std::cos(angle)),
        static_cast<float>(ray_length * std::sin(angle))};
      const Point2D base_endpoint =
        transformPoint(base_from_laser, laser_endpoint);
      observations.push_back(
        Keyframe::MapRayObservation{
          base_endpoint,
          endpoint_is_hit});
    }
    return observations;
  }

  void integrateKeyframeIntoMap(
    const Keyframe & keyframe,
    OccupancyGridMap & map) const
  {
    if (keyframe.map_rays.empty()) {
      return;
    }

    const Point2D map_sensor_origin =
      transformPoint(keyframe.pose, keyframe.sensor_origin);
    for (const auto & observation : keyframe.map_rays) {
      const Point2D map_endpoint =
        transformPoint(keyframe.pose, observation.endpoint);
      map.updateRay(
        map_sensor_origin,
        map_endpoint,
        observation.endpoint_is_hit);
    }
  }

  void integrateKeyframeIntoActiveMap(const Keyframe & keyframe)
  {
    integrateKeyframeIntoMap(keyframe, *occupancy_grid_map_);
    latest_map_stamp_ = keyframe.stamp;
    map_dirty_ = true;
  }

  void startMapRebuild()
  {
    pending_rebuild_map_ =
      std::make_unique<OccupancyGridMap>(map_parameters_);
    next_rebuild_keyframe_ = 0U;
    map_rebuild_started_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      get_logger(),
      "Started incremental occupancy grid rebuild for %zu keyframes",
      keyframes_.size());
  }

  void processMapRebuildBatch()
  {
    if (!pending_rebuild_map_) {
      return;
    }

    try {
      const std::size_t final_keyframe = std::min(
        next_rebuild_keyframe_ + map_rebuild_keyframes_per_cycle_,
        keyframes_.size());
      for (; next_rebuild_keyframe_ < final_keyframe;
        ++next_rebuild_keyframe_)
      {
        integrateKeyframeIntoMap(
          keyframes_[next_rebuild_keyframe_],
          *pending_rebuild_map_);
      }
    } catch (const std::exception & error) {
      pending_rebuild_map_.reset();
      RCLCPP_ERROR(
        get_logger(),
        "Aborted occupancy grid rebuild: %s",
        error.what());
      return;
    }
    if (next_rebuild_keyframe_ < keyframes_.size()) {
      return;
    }

    std::swap(occupancy_grid_map_, pending_rebuild_map_);
    pending_rebuild_map_.reset();
    latest_map_stamp_ = keyframes_.back().stamp;
    map_dirty_ = true;
    const double elapsed_milliseconds =
      std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - map_rebuild_started_).count();
    RCLCPP_INFO(
      get_logger(),
      "Completed occupancy grid rebuild from %zu optimized keyframes "
      "in %.1f ms",
      next_rebuild_keyframe_,
      elapsed_milliseconds);
    publishMapIfDirty();
  }

  void backgroundMaintenanceCallback()
  {
    applyCompletedLoopClosure();
    processMapRebuildBatch();
  }

  void publishMapIfDirty()
  {
    if (!map_dirty_) {
      return;
    }

    OccupancyGridSnapshot snapshot;
    try {
      snapshot = occupancy_grid_map_->snapshot();
    } catch (const std::exception & error) {
      map_dirty_ = false;
      RCLCPP_ERROR(
        get_logger(),
        "Failed to create occupancy grid snapshot: %s",
        error.what());
      return;
    }
    if (snapshot.data.empty()) {
      return;
    }

    nav_msgs::msg::OccupancyGrid map;
    map.header.stamp = latest_map_stamp_;
    map.header.frame_id = map_frame_;
    map.info.map_load_time = latest_map_stamp_;
    map.info.resolution = static_cast<float>(snapshot.resolution);
    map.info.width = static_cast<uint32_t>(snapshot.width);
    map.info.height = static_cast<uint32_t>(snapshot.height);
    map.info.origin.position.x =
      static_cast<double>(snapshot.origin_cell_x) * snapshot.resolution;
    map.info.origin.position.y =
      static_cast<double>(snapshot.origin_cell_y) * snapshot.resolution;
    map.info.origin.orientation.w = 1.0;
    map.data = snapshot.data;
    map_publisher_->publish(map);
    map_dirty_ = false;

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Published occupancy grid %u x %u (%zu observed cells)",
      map.info.width,
      map.info.height,
      occupancy_grid_map_->observedCellCount());
  }

  void publishEstimate(
    const builtin_interfaces::msg::Time & stamp,
    const Pose2D & odometry_pose,
    const std::vector<Point2D> & base_points)
  {
    const Pose2D map_from_odom =
      composePoses(estimated_pose_, inversePose(odometry_pose));
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = stamp;
    map_to_odom.header.frame_id = map_frame_;
    map_to_odom.child_frame_id = odom_frame_;
    map_to_odom.transform.translation.x = map_from_odom.x;
    map_to_odom.transform.translation.y = map_from_odom.y;
    map_to_odom.transform.rotation = yawToQuaternion(map_from_odom.yaw);
    transform_broadcaster_->sendTransform(map_to_odom);

    nav_msgs::msg::Odometry laser_odometry;
    laser_odometry.header.stamp = stamp;
    laser_odometry.header.frame_id = map_frame_;
    laser_odometry.child_frame_id = base_frame_;
    laser_odometry.pose.pose.position.x = estimated_pose_.x;
    laser_odometry.pose.pose.position.y = estimated_pose_.y;
    laser_odometry.pose.pose.orientation =
      yawToQuaternion(estimated_pose_.yaw);
    laser_odometry.twist.covariance[0] = -1.0;
    laser_odometry_publisher_->publish(laser_odometry);

    geometry_msgs::msg::PoseStamped path_pose;
    path_pose.header = laser_odometry.header;
    path_pose.pose = laser_odometry.pose.pose;
    path_.header = laser_odometry.header;
    path_.poses.push_back(path_pose);
    if (path_.poses.size() > static_cast<std::size_t>(maximum_path_poses_)) {
      path_.poses.erase(path_.poses.begin());
    }
    laser_path_publisher_->publish(path_);

    sensor_msgs::msg::PointCloud2 aligned_points;
    aligned_points.header = laser_odometry.header;
    aligned_points.height = 1U;
    aligned_points.width = static_cast<uint32_t>(base_points.size());
    aligned_points.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(aligned_points);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(base_points.size());

    sensor_msgs::PointCloud2Iterator<float> output_x(aligned_points, "x");
    sensor_msgs::PointCloud2Iterator<float> output_y(aligned_points, "y");
    sensor_msgs::PointCloud2Iterator<float> output_z(aligned_points, "z");
    for (const auto & base_point : base_points) {
      const Point2D aligned = transformPoint(estimated_pose_, base_point);
      *output_x = aligned.x;
      *output_y = aligned.y;
      *output_z = 0.0F;
      ++output_x;
      ++output_y;
      ++output_z;
    }
    aligned_points_publisher_->publish(aligned_points);
  }

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double minimum_range_;
  double maximum_range_;
  std::size_t point_stride_;
  double maximum_odom_age_;
  double minimum_translation_for_update_;
  double minimum_rotation_for_update_;
  std::size_t maximum_local_keyframes_;
  int64_t maximum_path_poses_;
  std::size_t map_ray_stride_;
  std::size_t map_rebuild_keyframes_per_cycle_;
  double sequential_translation_weight_;
  double sequential_rotation_weight_;
  bool loop_closure_enabled_;
  LoopClosureCandidateParameters loop_candidate_parameters_;
  std::size_t loop_closure_check_interval_;
  std::size_t minimum_loop_closure_interval_;
  std::size_t candidate_submap_half_width_;
  double loop_translation_weight_;
  double loop_rotation_weight_;
  PoseGraphOptimizationOptions pose_graph_optimization_options_;
  CorrelativeScanMatcherParameters matcher_parameters_;
  CorrelativeScanMatcherParameters loop_matcher_parameters_;
  OccupancyGridMapParameters map_parameters_;
  std::unique_ptr<OccupancyGridMap> occupancy_grid_map_;
  std::unique_ptr<OccupancyGridMap> pending_rebuild_map_;
  PoseGraph2D pose_graph_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
    laser_scan_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
    laser_odometry_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr laser_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
    pose_graph_path_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    aligned_points_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::TimerBase::SharedPtr map_publish_timer_;
  rclcpp::TimerBase::SharedPtr map_rebuild_timer_;

  std::deque<OdomSample> odom_samples_;
  std::vector<Keyframe> keyframes_;
  bool initialized_{false};
  Pose2D last_matched_odometry_pose_;
  Pose2D last_matched_pose_;
  Pose2D estimated_pose_;
  nav_msgs::msg::Path path_;
  nav_msgs::msg::Path pose_graph_path_;
  builtin_interfaces::msg::Time latest_map_stamp_;
  bool map_dirty_{false};
  std::size_t next_rebuild_keyframe_{0U};
  std::chrono::steady_clock::time_point map_rebuild_started_;
  std::optional<std::size_t> last_loop_closure_node_;
  std::optional<std::future<LoopClosureProcessingResult>>
  loop_closure_future_;
  std::chrono::steady_clock::time_point loop_closure_job_started_;
};

}  // namespace slam_robot_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<slam_robot_slam::ScanMatcherOdometryNode>());
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("scan_matcher_odometry"),
      "Fatal SLAM node error: %s",
      error.what());
    rclcpp::shutdown();
    return 1;
  }
}
