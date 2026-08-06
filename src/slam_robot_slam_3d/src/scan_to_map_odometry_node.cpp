#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include "slam_robot_slam_3d/local_submap.hpp"
#include "slam_robot_slam_3d/global_keyframe_map.hpp"
#include "slam_robot_slam_3d/loop_closure_verifier.hpp"
#include "slam_robot_slam_3d/match_failure_recovery.hpp"
#include "slam_robot_slam_3d/odometry_interpolator.hpp"
#include "slam_robot_slam_3d/scan_context_index.hpp"
#include "slam_robot_slam_3d/scan_to_map_matcher.hpp"
#include "slam_robot_slam_3d/se2_pose_graph_backend.hpp"

namespace slam_robot_slam_3d
{
namespace
{

// Variance published for a translation or yaw direction the geometry does not
// constrain. ScanToMapResult::translationCovariance requires it to stay at or
// above the nominal variance, which this node derives from the accepted RMSE,
// so maximum_rmse must remain below its square root. That bound is enforced at
// construction rather than left to fail per callback: the throw would surface
// only as a throttled error while /custom_slam_3d/laser_odom went silent.
constexpr double kUnobservableVariance = 0.25;

diagnostic_msgs::msg::KeyValue makeValue(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose & pose)
{
  const Eigen::Quaterniond rotation(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  if (!rotation.coeffs().allFinite() || rotation.norm() < 1.0e-6 ||
    !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
    !std::isfinite(pose.position.z))
  {
    throw std::invalid_argument("odometry pose is not finite");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  result.linear() = rotation.normalized().toRotationMatrix();
  return result;
}

Eigen::Isometry3d transformToEigen(
  const geometry_msgs::msg::Transform & transform)
{
  const Eigen::Quaterniond rotation(
    transform.rotation.w, transform.rotation.x,
    transform.rotation.y, transform.rotation.z);
  if (!rotation.coeffs().allFinite() || rotation.norm() < 1.0e-6 ||
    !std::isfinite(transform.translation.x) ||
    !std::isfinite(transform.translation.y) ||
    !std::isfinite(transform.translation.z))
  {
    throw std::invalid_argument("static sensor transform is not finite");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(
    transform.translation.x, transform.translation.y,
    transform.translation.z);
  result.linear() = rotation.normalized().toRotationMatrix();
  return result;
}

geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::Pose result;
  result.position.x = pose.translation().x();
  result.position.y = pose.translation().y();
  result.position.z = pose.translation().z();
  const Eigen::Quaterniond rotation(pose.rotation());
  result.orientation.x = rotation.x();
  result.orientation.y = rotation.y();
  result.orientation.z = rotation.z();
  result.orientation.w = rotation.w();
  return result;
}

double rotationMagnitude(const Eigen::Isometry3d & relative_pose)
{
  return Eigen::AngleAxisd(relative_pose.rotation()).angle();
}

Eigen::Isometry3d planarPose(const Eigen::Isometry3d & pose)
{
  const double yaw = std::atan2(pose.rotation()(1, 0), pose.rotation()(0, 0));
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation().x() = pose.translation().x();
  result.translation().y() = pose.translation().y();
  result.linear() =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return result;
}

}  // namespace

class ScanToMapOdometryNode : public rclcpp::Node
{
public:
  ScanToMapOdometryNode()
  : Node("scan_to_map_odometry_3d"),
    matcher_(declareMatcherParameters()),
    local_submap_(declareSubmapParameters()),
    scan_context_index_(declareScanContextParameters()),
    loop_closure_verifier_(declareLoopClosureVerifierParameters()),
    pose_graph_backend_(declarePoseGraphBackendParameters()),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>(
      "front_end.input_topic", "/custom_slam_3d/points_filtered");
    odom_topic_ = declare_parameter<std::string>(
      "front_end.odom_topic", "/odom");
    base_frame_ = declare_parameter<std::string>(
      "front_end.base_frame", "base_footprint");
    local_frame_ = declare_parameter<std::string>(
      "front_end.local_frame", "custom_slam_3d_odom");
    map_frame_ = declare_parameter<std::string>("pose_graph.map_frame", "map");
    maximum_odom_age_ = declare_parameter<double>(
      "front_end.maximum_odom_age", 0.05);
    force_planar_motion_ = declare_parameter<bool>(
      "front_end.force_planar_motion", true);
    keyframe_translation_ = declare_parameter<double>(
      "front_end.keyframe_translation", 0.25);
    keyframe_rotation_ = declare_parameter<double>(
      "front_end.keyframe_rotation", 0.15);
    const auto buffer_size = declare_parameter<int>(
      "front_end.maximum_odom_buffer_size", 200);
    const auto pending_cloud_limit = declare_parameter<int>(
      "front_end.maximum_pending_clouds", 5);
    const auto failure_limit = declare_parameter<int>(
      "front_end.maximum_consecutive_match_failures", 5);
    if (input_topic_.empty() || odom_topic_.empty() || base_frame_.empty() ||
      local_frame_.empty() || map_frame_.empty() || !std::isfinite(maximum_odom_age_) ||
      maximum_odom_age_ <= 0.0 || !std::isfinite(keyframe_translation_) ||
      keyframe_translation_ <= 0.0 || !std::isfinite(keyframe_rotation_) ||
      keyframe_rotation_ <= 0.0 || buffer_size <= 1 ||
      pending_cloud_limit <= 0 || failure_limit <= 0)
    {
      throw std::invalid_argument("front-end parameters are invalid");
    }
    maximum_odom_buffer_size_ = static_cast<std::size_t>(buffer_size);
    maximum_pending_clouds_ = static_cast<std::size_t>(pending_cloud_limit);
    match_failure_recovery_ = std::make_unique<MatchFailureRecovery>(
      static_cast<std::size_t>(failure_limit));

    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "/custom_slam_3d/laser_odom", 10);
    registered_scan_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      "/custom_slam_3d/registered_scan", rclcpp::SensorDataQoS());
    local_map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/custom_slam_3d/local_map", rclcpp::QoS(1).transient_local());
    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/custom_slam_3d/front_end_diagnostics", 10);
    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(200).reliable(),
      std::bind(
        &ScanToMapOdometryNode::odomCallback, this,
        std::placeholders::_1));
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(
        &ScanToMapOdometryNode::cloudCallback, this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "3D GICP front end %s + %s -> /custom_slam_3d/laser_odom; publishing %s -> %s",
      input_topic_.c_str(), odom_topic_.c_str(), map_frame_.c_str(), local_frame_.c_str());
  }

private:
  ScanToMapMatcherParameters declareMatcherParameters()
  {
    ScanToMapMatcherParameters parameters;
    parameters.maximum_iterations = declare_parameter<int>(
      "matcher.maximum_iterations", parameters.maximum_iterations);
    parameters.maximum_optimizer_iterations = declare_parameter<int>(
      "matcher.maximum_optimizer_iterations",
      parameters.maximum_optimizer_iterations);
    parameters.correspondence_randomness = declare_parameter<int>(
      "matcher.correspondence_randomness",
      parameters.correspondence_randomness);
    parameters.maximum_correspondence_distance = declare_parameter<double>(
      "matcher.maximum_correspondence_distance",
      parameters.maximum_correspondence_distance);
    parameters.transformation_epsilon = declare_parameter<double>(
      "matcher.transformation_epsilon", parameters.transformation_epsilon);
    parameters.rotation_epsilon = declare_parameter<double>(
      "matcher.rotation_epsilon", parameters.rotation_epsilon);
    parameters.euclidean_fitness_epsilon = declare_parameter<double>(
      "matcher.euclidean_fitness_epsilon",
      parameters.euclidean_fitness_epsilon);
    const auto minimum_points = declare_parameter<int>(
      "matcher.minimum_points", static_cast<int>(parameters.minimum_points));
    const auto minimum_correspondences = declare_parameter<int>(
      "matcher.minimum_correspondences",
      static_cast<int>(parameters.minimum_correspondences));
    if (minimum_points <= 0 || minimum_correspondences <= 0) {
      throw std::invalid_argument("matcher point limits must be positive");
    }
    parameters.minimum_points = static_cast<std::size_t>(minimum_points);
    parameters.minimum_correspondences =
      static_cast<std::size_t>(minimum_correspondences);
    parameters.maximum_rmse = declare_parameter<double>(
      "matcher.maximum_rmse", parameters.maximum_rmse);
    if (parameters.maximum_rmse * parameters.maximum_rmse >=
      kUnobservableVariance)
    {
      throw std::invalid_argument(
              "matcher.maximum_rmse must stay below the square root of the "
              "unobservable variance so an accepted match never publishes a "
              "nominal variance above it");
    }
    parameters.maximum_correction_translation = declare_parameter<double>(
      "matcher.maximum_correction_translation",
      parameters.maximum_correction_translation);
    parameters.maximum_correction_rotation = declare_parameter<double>(
      "matcher.maximum_correction_rotation",
      parameters.maximum_correction_rotation);
    parameters.minimum_translation_information_ratio =
      declare_parameter<double>(
      "matcher.minimum_translation_information_ratio",
      parameters.minimum_translation_information_ratio);
    parameters.full_suppression_translation_information_ratio =
      declare_parameter<double>(
      "matcher.full_suppression_translation_information_ratio",
      parameters.full_suppression_translation_information_ratio);
    parameters.minimum_translation_information = declare_parameter<double>(
      "matcher.minimum_translation_information",
      parameters.minimum_translation_information);
    parameters.full_suppression_translation_information =
      declare_parameter<double>(
      "matcher.full_suppression_translation_information",
      parameters.full_suppression_translation_information);
    parameters.minimum_planar_information = declare_parameter<double>(
      "matcher.minimum_planar_information",
      parameters.minimum_planar_information);
    parameters.minimum_yaw_information = declare_parameter<double>(
      "matcher.minimum_yaw_information",
      parameters.minimum_yaw_information);
    parameters.degeneracy_handling_enabled = declare_parameter<bool>(
      "matcher.degeneracy_handling_enabled",
      parameters.degeneracy_handling_enabled);
    return parameters;
  }

  LocalSubmapParameters declareSubmapParameters()
  {
    LocalSubmapParameters parameters;
    const auto maximum_keyframes = declare_parameter<int>(
      "front_end.local_submap.maximum_keyframes",
      static_cast<int>(parameters.maximum_keyframes));
    if (maximum_keyframes <= 0) {
      throw std::invalid_argument("maximum_keyframes must be positive");
    }
    parameters.maximum_keyframes = static_cast<std::size_t>(maximum_keyframes);
    parameters.voxel_leaf_size = declare_parameter<double>(
      "front_end.local_submap.voxel_leaf_size",
      parameters.voxel_leaf_size);
    return parameters;
  }

  ScanContextParameters declareScanContextParameters()
  {
    ScanContextParameters parameters;
    parameters.maximum_radius = declare_parameter<double>(
      "loop_closure.scan_context.maximum_radius", parameters.maximum_radius);
    const auto radial_bins = declare_parameter<int>(
      "loop_closure.scan_context.radial_bins",
      static_cast<int>(parameters.radial_bins));
    const auto angular_bins = declare_parameter<int>(
      "loop_closure.scan_context.angular_bins",
      static_cast<int>(parameters.angular_bins));
    const auto minimum_keyframe_separation = declare_parameter<int>(
      "loop_closure.scan_context.minimum_keyframe_separation",
      static_cast<int>(parameters.minimum_keyframe_separation));
    const auto ring_key_candidate_count = declare_parameter<int>(
      "loop_closure.scan_context.ring_key_candidate_count",
      static_cast<int>(parameters.ring_key_candidate_count));
    const auto maximum_candidates = declare_parameter<int>(
      "loop_closure.scan_context.maximum_candidates",
      static_cast<int>(parameters.maximum_candidates));
    if (radial_bins <= 0 || angular_bins <= 0 ||
      minimum_keyframe_separation <= 0 || ring_key_candidate_count <= 0 ||
      maximum_candidates <= 0)
    {
      throw std::invalid_argument("scan context count parameters must be positive");
    }
    parameters.radial_bins = static_cast<std::size_t>(radial_bins);
    parameters.angular_bins = static_cast<std::size_t>(angular_bins);
    parameters.minimum_keyframe_separation =
      static_cast<std::size_t>(minimum_keyframe_separation);
    parameters.minimum_travel_distance = declare_parameter<double>(
      "loop_closure.scan_context.minimum_travel_distance",
      parameters.minimum_travel_distance);
    parameters.ring_key_candidate_count =
      static_cast<std::size_t>(ring_key_candidate_count);
    parameters.maximum_candidates = static_cast<std::size_t>(maximum_candidates);
    return parameters;
  }

  LoopClosureVerifierParameters declareLoopClosureVerifierParameters()
  {
    LoopClosureVerifierParameters parameters;
    parameters.matcher = matcher_.parameters();
    parameters.matcher.maximum_correction_translation = declare_parameter<double>(
      "loop_closure.verification.maximum_correction_translation", 3.0);
    parameters.matcher.maximum_correction_rotation = declare_parameter<double>(
      "loop_closure.verification.maximum_correction_rotation", 1.0);
    parameters.matcher.degeneracy_handling_enabled = false;
    const auto submap_neighbors = declare_parameter<int>(
      "loop_closure.verification.submap_neighbor_keyframes",
      static_cast<int>(parameters.submap_neighbor_keyframes));
    const auto verification_candidates = declare_parameter<int>(
      "loop_closure.verification.maximum_candidates", 3);
    if (submap_neighbors <= 0 || verification_candidates <= 0) {
      throw std::invalid_argument("loop closure verification counts must be positive");
    }
    parameters.submap_neighbor_keyframes =
      static_cast<std::size_t>(submap_neighbors);
    maximum_loop_verification_candidates_ =
      static_cast<std::size_t>(verification_candidates);
    parameters.submap_voxel_leaf_size = declare_parameter<double>(
      "loop_closure.verification.submap_voxel_leaf_size",
      parameters.submap_voxel_leaf_size);
    parameters.minimum_overlap_ratio = declare_parameter<double>(
      "loop_closure.verification.minimum_overlap_ratio",
      parameters.minimum_overlap_ratio);
    return parameters;
  }

  Se2PoseGraphBackendParameters declarePoseGraphBackendParameters()
  {
    Se2PoseGraphBackendParameters parameters;
    parameters.maximum_iterations = declare_parameter<int>(
      "pose_graph.maximum_iterations", parameters.maximum_iterations);
    parameters.loop_closure_huber_scale = declare_parameter<double>(
      "pose_graph.loop_closure_huber_scale", parameters.loop_closure_huber_scale);
    parameters.sequential_translation_weight = declare_parameter<double>(
      "pose_graph.sequential_translation_weight", parameters.sequential_translation_weight);
    parameters.sequential_rotation_weight = declare_parameter<double>(
      "pose_graph.sequential_rotation_weight", parameters.sequential_rotation_weight);
    parameters.loop_translation_weight = declare_parameter<double>(
      "pose_graph.loop_translation_weight", parameters.loop_translation_weight);
    parameters.loop_rotation_weight = declare_parameter<double>(
      "pose_graph.loop_rotation_weight", parameters.loop_rotation_weight);
    const auto minimum_interval = declare_parameter<int>(
      "pose_graph.minimum_keyframe_interval", 30);
    if (minimum_interval <= 0) {
      throw std::invalid_argument("pose graph minimum keyframe interval must be positive");
    }
    minimum_pose_graph_keyframe_interval_ =
      static_cast<std::size_t>(minimum_interval);
    return parameters;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    try {
      (void)poseToEigen(message->pose.pose);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Rejected odometry input: %s", error.what());
      return;
    }
    odom_buffer_.push_back(*message);
    while (odom_buffer_.size() > maximum_odom_buffer_size_) {
      odom_buffer_.pop_front();
    }
    processPendingClouds();
  }

  bool pendingCloudIsStale(const rclcpp::Time & cloud_time) const
  {
    return std::any_of(
      odom_buffer_.begin(), odom_buffer_.end(),
      [&](const auto & sample) {
        return (rclcpp::Time(sample.header.stamp) - cloud_time).seconds() >
               maximum_odom_age_;
      });
  }

  void processPendingClouds()
  {
    while (!pending_clouds_.empty()) {
      const auto & pending_cloud = pending_clouds_.front();
      const auto odometry = interpolateOdometry(
        odom_buffer_,
        rclcpp::Time(pending_cloud->header.stamp), maximum_odom_age_);
      if (!odometry.has_value()) {
        if (pendingCloudIsStale(rclcpp::Time(pending_cloud->header.stamp))) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Dropping a 3D scan without a valid odometry time bracket");
          pending_clouds_.pop_front();
          continue;
        }
        return;
      }
      auto cloud = pending_cloud;
      pending_clouds_.pop_front();
      processCloud(cloud, *odometry);
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    pending_clouds_.push_back(message);
    if (pending_clouds_.size() > maximum_pending_clouds_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Replacing a 3D scan still waiting for synchronized odometry");
      pending_clouds_.pop_front();
    }
    processPendingClouds();
  }

  void processCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & message,
    const nav_msgs::msg::Odometry & odometry)
  {
    applyLoopClosureResult();
    applyPoseGraphResult();
    const auto started = std::chrono::steady_clock::now();
    try {
      const auto base_to_lidar_message = tf_buffer_.lookupTransform(
        base_frame_, message->header.frame_id, tf2::TimePointZero);
      const Eigen::Isometry3d base_to_lidar =
        transformToEigen(base_to_lidar_message.transform);
      const Eigen::Isometry3d odom_pose = poseToEigen(odometry.pose.pose);

      pcl::PointCloud<pcl::PointXYZI> scan;
      pcl::fromROSMsg(*message, scan);
      if (!initialized_) {
        estimated_base_pose_ = Eigen::Isometry3d::Identity();
        last_keyframe_base_pose_ = estimated_base_pose_;
        last_odom_pose_ = odom_pose;
        const Eigen::Isometry3d lidar_pose =
          estimated_base_pose_ * base_to_lidar;
        local_submap_.addKeyframe(scan, lidar_pose);
        addGlobalKeyframe(
          message->header, scan, estimated_base_pose_, odom_pose,
          base_to_lidar, nullptr);
        initialized_ = true;
        publishOutputs(
          *message, odometry, scan, lidar_pose, nullptr, true, false, started);
        return;
      }

      const Eigen::Isometry3d odom_delta =
        last_odom_pose_.inverse() * odom_pose;
      Eigen::Isometry3d predicted_base_pose =
        estimated_base_pose_ * odom_delta;
      if (force_planar_motion_) {
        predicted_base_pose = planarPose(predicted_base_pose);
      }
      const Eigen::Isometry3d predicted_lidar_pose =
        predicted_base_pose * base_to_lidar;
      const auto match_result = matcher_.match(
        scan, local_submap_.cloud(), local_submap_.version(), predicted_lidar_pose);

      Eigen::Isometry3d lidar_pose = predicted_lidar_pose;
      bool keyframe_added = false;
      bool submap_reinitialized = false;
      if (match_result.success()) {
        (void)match_failure_recovery_->observe(ScanToMapStatus::kSuccess);
        estimated_base_pose_ = match_result.pose * base_to_lidar.inverse();
        if (force_planar_motion_) {
          estimated_base_pose_ = planarPose(estimated_base_pose_);
        }
        lidar_pose = estimated_base_pose_ * base_to_lidar;
        const Eigen::Isometry3d keyframe_delta =
          last_keyframe_base_pose_.inverse() * estimated_base_pose_;
        if (keyframe_delta.translation().norm() >= keyframe_translation_ ||
          rotationMagnitude(keyframe_delta) >= keyframe_rotation_)
        {
          local_submap_.addKeyframe(scan, lidar_pose);
          last_keyframe_base_pose_ = estimated_base_pose_;
          keyframe_added = true;
        }
      } else {
        estimated_base_pose_ = predicted_base_pose;
        if (match_failure_recovery_->observe(match_result.status)) {
          local_submap_.clear();
          local_submap_.addKeyframe(scan, lidar_pose);
          last_keyframe_base_pose_ = estimated_base_pose_;
          keyframe_added = true;
          submap_reinitialized = true;
          RCLCPP_WARN(
            get_logger(),
            "Reinitialized the 3D local submap after consecutive match failures");
        }
      }
      if (keyframe_added) {
        addGlobalKeyframe(
          message->header, scan, estimated_base_pose_, odom_pose,
          base_to_lidar, &match_result);
      }
      last_odom_pose_ = odom_pose;
      publishOutputs(
        *message, odometry, scan, lidar_pose, &match_result,
        keyframe_added, submap_reinitialized, started);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for 3D LiDAR static transform: %s", error.what());
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "3D front-end callback failed: %s", error.what());
    }
  }

  void publishOutputs(
    const sensor_msgs::msg::PointCloud2 & input_message,
    const nav_msgs::msg::Odometry & input_odometry,
    const pcl::PointCloud<pcl::PointXYZI> & scan,
    const Eigen::Isometry3d & lidar_pose,
    const ScanToMapResult * match_result,
    bool keyframe_added,
    bool submap_reinitialized,
    const std::chrono::steady_clock::time_point & started)
  {
    nav_msgs::msg::Odometry odometry;
    odometry.header = input_message.header;
    odometry.header.frame_id = local_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose = eigenToPose(estimated_base_pose_);
    odometry.twist = input_odometry.twist;
    const Eigen::Matrix<double, 6, 6> pose_covariance =
      makePoseCovariance(match_result);
    const Eigen::Matrix2d translation_covariance =
      pose_covariance.block<2, 2>(0, 0);
    for (std::size_t row = 0U; row < 6U; ++row) {
      for (std::size_t column = 0U; column < 6U; ++column) {
        odometry.pose.covariance[row * 6U + column] =
          pose_covariance(static_cast<Eigen::Index>(row),
          static_cast<Eigen::Index>(column));
      }
    }
    const double yaw_variance = pose_covariance(5, 5);
    odometry_publisher_->publish(odometry);
    geometry_msgs::msg::TransformStamped map_to_local;
    map_to_local.header = input_message.header;
    map_to_local.header.frame_id = map_frame_;
    map_to_local.child_frame_id = local_frame_;
    map_to_local.transform.translation.x = map_from_local_.translation().x();
    map_to_local.transform.translation.y = map_from_local_.translation().y();
    map_to_local.transform.translation.z = 0.0;
    const Eigen::Quaterniond map_rotation(map_from_local_.rotation());
    map_to_local.transform.rotation.x = map_rotation.x();
    map_to_local.transform.rotation.y = map_rotation.y();
    map_to_local.transform.rotation.z = map_rotation.z();
    map_to_local.transform.rotation.w = map_rotation.w();
    transform_broadcaster_->sendTransform(map_to_local);

    pcl::PointCloud<pcl::PointXYZI> registered_scan;
    pcl::transformPointCloud(
      scan, registered_scan, lidar_pose.matrix().cast<float>());
    sensor_msgs::msg::PointCloud2 registered_message;
    pcl::toROSMsg(registered_scan, registered_message);
    registered_message.header = input_message.header;
    registered_message.header.frame_id = local_frame_;
    registered_scan_publisher_->publish(registered_message);

    if (keyframe_added) {
      sensor_msgs::msg::PointCloud2 map_message;
      pcl::toROSMsg(local_submap_.cloud(), map_message);
      map_message.header = input_message.header;
      map_message.header.frame_id = local_frame_;
      local_map_publisher_->publish(map_message);
    }
    publishDiagnostics(
      input_message.header, match_result, keyframe_added,
      submap_reinitialized, translation_covariance, yaw_variance, started);
  }

  Eigen::Matrix<double, 6, 6> makePoseCovariance(
    const ScanToMapResult * match_result) const
  {
    const double position_variance = match_result != nullptr &&
      match_result->success() ?
      std::max(1.0e-4, match_result->rmse * match_result->rmse) :
      kUnobservableVariance;
    Eigen::Matrix2d translation_covariance =
      position_variance * Eigen::Matrix2d::Identity();
    if (match_result != nullptr && match_result->success()) {
      translation_covariance = match_result->translationCovariance(
        position_variance, kUnobservableVariance);
    }
    Eigen::Matrix<double, 6, 6> result =
      Eigen::Matrix<double, 6, 6>::Zero();
    result(0, 0) = translation_covariance(0, 0);
    result(0, 1) = translation_covariance(0, 1);
    result(1, 0) = translation_covariance(1, 0);
    result(1, 1) = translation_covariance(1, 1);
    result(2, 2) = force_planar_motion_ ? 1.0e-4 : position_variance;
    result(3, 3) = force_planar_motion_ ? 1.0e-4 : 0.05;
    result(4, 4) = force_planar_motion_ ? 1.0e-4 : 0.05;
    result(5, 5) = match_result != nullptr && match_result->success() &&
      !match_result->yaw_degenerate ? 0.01 : kUnobservableVariance;
    return result;
  }

  void addGlobalKeyframe(
    const std_msgs::msg::Header & header,
    const pcl::PointCloud<pcl::PointXYZI> & scan,
    const Eigen::Isometry3d & front_end_base_pose,
    const Eigen::Isometry3d & odom_base_pose,
    const Eigen::Isometry3d & base_to_sensor,
    const ScanToMapResult * match_result)
  {
    if (has_last_global_keyframe_) {
      const Eigen::Vector2d displacement =
        front_end_base_pose.translation().head<2>() -
        last_global_keyframe_base_pose_.translation().head<2>();
      global_accumulated_distance_ += displacement.norm();
    }
    GlobalKeyframe keyframe;
    keyframe.stamp = rclcpp::Time(header.stamp);
    keyframe.filtered_scan =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(scan);
    keyframe.front_end_base_pose = front_end_base_pose;
    keyframe.odom_base_pose = odom_base_pose;
    keyframe.base_to_sensor = base_to_sensor;
    keyframe.pose_covariance = makePoseCovariance(match_result);
    keyframe.accumulated_distance = global_accumulated_distance_;
    keyframe.match_accepted = match_result == nullptr || match_result->success();
    keyframe.translation_degenerate =
      match_result != nullptr && match_result->translation_degenerate;
    keyframe.planar_degenerate =
      match_result != nullptr && match_result->planar_degenerate;
    keyframe.yaw_degenerate =
      match_result != nullptr && match_result->yaw_degenerate;
    keyframe.correspondence_count =
      match_result == nullptr ? 0U : match_result->correspondence_count;
    keyframe.rmse = match_result == nullptr ? 0.0 : match_result->rmse;
    keyframe.id = global_keyframes_.add(keyframe);
    pending_loop_keyframes_.push_back(std::move(keyframe));
    startLoopClosureTask();
    last_global_keyframe_base_pose_ = front_end_base_pose;
    has_last_global_keyframe_ = true;
  }

  struct PoseGraphResult
  {
    Se2PoseGraphBackendResult optimization;
    std::vector<Se2LoopConstraint> constraints;
  };

  struct LoopClosureTaskResult
  {
    std::size_t scan_context_index_size{0U};
    std::vector<ScanContextCandidate> candidates;
    std::vector<LoopClosureVerificationResult> verifications;
    std::vector<Se2LoopConstraint> accepted_constraints;
  };

  void startLoopClosureTask()
  {
    if (loop_closure_future_.has_value() || pending_loop_keyframes_.empty()) {
      return;
    }
    const GlobalKeyframe current = std::move(pending_loop_keyframes_.front());
    pending_loop_keyframes_.pop_front();
    const auto keyframes = global_keyframes_.snapshot();
    loop_closure_future_.emplace(std::async(std::launch::async,
      [this, current, keyframes]() {
        LoopClosureTaskResult result;
        result.candidates = scan_context_index_.addAndQuery(current);
        result.scan_context_index_size = scan_context_index_.size();
        const std::size_t candidate_count = std::min(
          maximum_loop_verification_candidates_, result.candidates.size());
        result.verifications.reserve(candidate_count);
        for (std::size_t index = 0U; index < candidate_count; ++index) {
          result.verifications.push_back(loop_closure_verifier_.verify(
            keyframes, current.id, result.candidates[index]));
        }
        for (const auto & verification : result.verifications) {
          if (!verification.accepted()) {
            continue;
          }
          const auto & historical = keyframes[verification.candidate_keyframe_id];
          const auto & latest = keyframes[current.id];
          const Eigen::Isometry3d current_base_pose =
          verification.current_sensor_pose * latest.base_to_sensor.inverse();
          result.accepted_constraints.push_back({
            verification.candidate_keyframe_id, current.id,
            historical.front_end_base_pose.inverse() * current_base_pose});
        }
        return result;
      }));
  }

  void applyLoopClosureResult()
  {
    if (!loop_closure_future_.has_value() ||
      loop_closure_future_->wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    {
      return;
    }
    try {
      const auto result = loop_closure_future_->get();
      loop_closure_future_.reset();
      last_scan_context_candidates_ = result.candidates;
      last_loop_verification_results_ = result.verifications;
      scan_context_index_size_ = result.scan_context_index_size;
      pending_loop_constraints_.insert(
        pending_loop_constraints_.end(), result.accepted_constraints.begin(),
        result.accepted_constraints.end());
    } catch (const std::exception & error) {
      loop_closure_future_.reset();
      ++loop_closure_failure_count_;
      RCLCPP_ERROR(get_logger(), "Discarded loop-closure worker result: %s", error.what());
    }
    startPoseGraphOptimization(global_keyframes_.snapshot());
    startLoopClosureTask();
  }

  void startPoseGraphOptimization(const std::vector<GlobalKeyframe> & keyframes)
  {
    if (pose_graph_future_.has_value() || pending_loop_constraints_.empty()) {
      return;
    }
    const std::size_t latest_keyframe_id = keyframes.back().id;
    if (has_pose_graph_submission_ &&
      latest_keyframe_id - last_pose_graph_submission_keyframe_id_ <
      minimum_pose_graph_keyframe_interval_)
    {
      return;
    }
    auto constraints = committed_loop_constraints_;
    constraints.insert(constraints.end(), pending_loop_constraints_.begin(),
      pending_loop_constraints_.end());
    pending_loop_constraints_.clear();
    last_pose_graph_submission_keyframe_id_ = latest_keyframe_id;
    has_pose_graph_submission_ = true;
    pose_graph_future_.emplace(std::async(std::launch::async,
      [this, keyframes, constraints = std::move(constraints)]() mutable {
        return PoseGraphResult{pose_graph_backend_.optimize(keyframes, constraints),
        std::move(constraints)};
      }));
  }

  void applyPoseGraphResult()
  {
    if (!pose_graph_future_.has_value() ||
      pose_graph_future_->wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    {
      return;
    }
    try {
      const auto result = pose_graph_future_->get();
      pose_graph_future_.reset();
      if (result.optimization.success &&
        result.optimization.snapshot_keyframe_count <= global_keyframes_.size())
      {
        committed_loop_constraints_ = result.constraints;
        map_from_local_ = result.optimization.map_from_local;
        ++pose_graph_commit_count_;
      } else {
        ++pose_graph_discard_count_;
      }
    } catch (const std::exception & error) {
      pose_graph_future_.reset();
      ++pose_graph_failure_count_;
      RCLCPP_ERROR(get_logger(), "Discarded pose graph worker result: %s", error.what());
    }
    startPoseGraphOptimization(global_keyframes_.snapshot());
  }

  void publishDiagnostics(
    const std_msgs::msg::Header & header,
    const ScanToMapResult * result,
    bool keyframe_added,
    bool submap_reinitialized,
    const Eigen::Matrix2d & translation_covariance,
    double yaw_variance,
    const std::chrono::steady_clock::time_point & started)
  {
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header = header;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "custom_slam_3d/scan_to_map_front_end";
    status.hardware_id = header.frame_id;
    const bool accepted = result == nullptr || result->success();
    status.level = accepted ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = result == nullptr ? "initialized" : toString(result->status);
    status.values.push_back(makeValue(
      "match_accepted", accepted ? "true" : "false"));
    status.values.push_back(makeValue(
      "correspondences",
      std::to_string(result == nullptr ? 0U : result->correspondence_count)));
    status.values.push_back(makeValue(
      "rmse", std::to_string(result == nullptr ? 0.0 : result->rmse)));
    status.values.push_back(makeValue(
      "correction_translation",
      std::to_string(result == nullptr ? 0.0 : result->correction_translation)));
    status.values.push_back(makeValue(
      "correction_rotation",
      std::to_string(result == nullptr ? 0.0 : result->correction_rotation)));
    status.values.push_back(makeValue(
      "applied_correction_translation",
      std::to_string(
        result == nullptr ? 0.0 : result->applied_correction_translation)));
    status.values.push_back(makeValue(
      "applied_correction_rotation",
      std::to_string(
        result == nullptr ? 0.0 : result->applied_correction_rotation)));
    status.values.push_back(makeValue(
      "observability_correspondences",
      std::to_string(
        result == nullptr ? 0U : result->observability_correspondences)));
    status.values.push_back(makeValue(
      "translation_information_min",
      std::to_string(
        result == nullptr ? 0.0 :
        result->translation_information_eigenvalues.minCoeff())));
    status.values.push_back(makeValue(
      "translation_information_max",
      std::to_string(
        result == nullptr ? 0.0 :
        result->translation_information_eigenvalues.maxCoeff())));
    status.values.push_back(makeValue(
      "translation_information_ratio",
      std::to_string(
        result == nullptr ? 0.0 : result->translation_information_ratio)));
    status.values.push_back(makeValue(
      "planar_information_min",
      std::to_string(
        result == nullptr ? 0.0 :
        result->planar_information_eigenvalues.minCoeff())));
    status.values.push_back(makeValue(
      "yaw_information",
      std::to_string(result == nullptr ? 0.0 : result->yaw_information)));
    status.values.push_back(makeValue(
      "translation_observable_rank",
      std::to_string(
        result == nullptr ? 0 : result->translation_observable_rank)));
    status.values.push_back(makeValue(
      "translation_degenerate",
      result != nullptr && result->translation_degenerate ? "true" : "false"));
    status.values.push_back(makeValue(
      "planar_degenerate",
      result != nullptr && result->planar_degenerate ? "true" : "false"));
    status.values.push_back(makeValue(
      "yaw_degenerate",
      result != nullptr && result->yaw_degenerate ? "true" : "false"));
    status.values.push_back(makeValue(
      "degenerate",
      result != nullptr && result->degenerate ? "true" : "false"));
    status.values.push_back(makeValue(
      "degeneracy_handling_applied",
      result != nullptr && result->degeneracy_handling_applied ?
      "true" : "false"));
    status.values.push_back(makeValue(
      "weak_translation_direction_x",
      std::to_string(
        result == nullptr ? 1.0 : result->weak_translation_direction.x())));
    status.values.push_back(makeValue(
      "weak_translation_direction_y",
      std::to_string(
        result == nullptr ? 0.0 : result->weak_translation_direction.y())));
    status.values.push_back(makeValue(
      "weak_translation_correction_scale",
      std::to_string(
        result == nullptr ? 1.0 :
        result->weak_translation_correction_scale)));
    status.values.push_back(makeValue(
      "translation_covariance_xx",
      std::to_string(translation_covariance(0, 0))));
    status.values.push_back(makeValue(
      "translation_covariance_xy",
      std::to_string(translation_covariance(0, 1))));
    status.values.push_back(makeValue(
      "translation_covariance_yy",
      std::to_string(translation_covariance(1, 1))));
    status.values.push_back(makeValue(
      "yaw_variance", std::to_string(yaw_variance)));
    status.values.push_back(makeValue(
      "target_cache_reused",
      result != nullptr && result->target_cache_reused ? "true" : "false"));
    status.values.push_back(makeValue(
      "keyframe_added", keyframe_added ? "true" : "false"));
    status.values.push_back(makeValue(
      "consecutive_match_failures",
      std::to_string(match_failure_recovery_->consecutiveFailures())));
    status.values.push_back(makeValue(
      "submap_reinitializations",
      std::to_string(match_failure_recovery_->reinitializationCount())));
    status.values.push_back(makeValue(
      "submap_reinitialized", submap_reinitialized ? "true" : "false"));
    status.values.push_back(makeValue(
      "local_keyframes", std::to_string(local_submap_.keyframeCount())));
    status.values.push_back(makeValue(
      "local_map_points", std::to_string(local_submap_.cloud().size())));
    status.values.push_back(makeValue(
      "global_keyframes", std::to_string(global_keyframes_.size())));
    status.values.push_back(makeValue(
      "global_keyframe_points", std::to_string(global_keyframes_.pointCount())));
    status.values.push_back(makeValue(
      "scan_context_index_size", std::to_string(scan_context_index_size_)));
    status.values.push_back(makeValue(
      "loop_closure_worker_active",
      loop_closure_future_.has_value() ? "true" : "false"));
    status.values.push_back(makeValue(
      "loop_closure_pending_keyframes",
      std::to_string(pending_loop_keyframes_.size())));
    status.values.push_back(makeValue(
      "loop_closure_failures", std::to_string(loop_closure_failure_count_)));
    status.values.push_back(makeValue(
      "loop_candidate_count",
      std::to_string(last_scan_context_candidates_.size())));
    const ScanContextCandidate * best_candidate =
      last_scan_context_candidates_.empty() ? nullptr :
      &last_scan_context_candidates_.front();
    status.values.push_back(makeValue(
      "loop_best_candidate_id",
      std::to_string(best_candidate == nullptr ? -1 :
      static_cast<long long>(best_candidate->keyframe_id))));
    status.values.push_back(makeValue(
      "loop_best_descriptor_distance",
      std::to_string(best_candidate == nullptr ? -1.0 :
      best_candidate->descriptor_distance)));
    status.values.push_back(makeValue(
      "loop_best_ring_key_distance",
      std::to_string(best_candidate == nullptr ? -1.0 :
      best_candidate->ring_key_distance)));
    status.values.push_back(makeValue(
      "loop_best_predicted_yaw",
      std::to_string(best_candidate == nullptr ? 0.0 :
      best_candidate->predicted_yaw)));
    const auto accepted_loop_count = static_cast<std::size_t>(std::count_if(
      last_loop_verification_results_.begin(),
      last_loop_verification_results_.end(),
        [](const auto & result) {return result.accepted();}));
    status.values.push_back(makeValue(
      "loop_verified_candidate_count",
      std::to_string(last_loop_verification_results_.size())));
    status.values.push_back(makeValue(
      "loop_accepted_candidate_count", std::to_string(accepted_loop_count)));
    const LoopClosureVerificationResult * best_verification =
      last_loop_verification_results_.empty() ? nullptr :
      &last_loop_verification_results_.front();
    status.values.push_back(makeValue(
      "loop_best_verification_status",
      best_verification == nullptr ? "none" :
      toString(best_verification->status)));
    status.values.push_back(makeValue(
      "loop_best_overlap_ratio",
      std::to_string(best_verification == nullptr ? 0.0 :
      best_verification->overlap_ratio)));
    status.values.push_back(makeValue(
      "pose_graph_worker_active", pose_graph_future_.has_value() ? "true" : "false"));
    status.values.push_back(makeValue(
      "pose_graph_commits", std::to_string(pose_graph_commit_count_)));
    status.values.push_back(makeValue(
      "pose_graph_discards", std::to_string(pose_graph_discard_count_)));
    status.values.push_back(makeValue(
      "pose_graph_failures", std::to_string(pose_graph_failure_count_)));
    status.values.push_back(makeValue(
      "pose_graph_pending_constraints",
      std::to_string(pending_loop_constraints_.size())));
    const double elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    status.values.push_back(makeValue("processing_ms", std::to_string(elapsed)));
    message.status.push_back(std::move(status));
    diagnostics_publisher_->publish(message);
  }

  ScanToMapMatcher matcher_;
  LocalSubmap local_submap_;
  GlobalKeyframeMap global_keyframes_;
  ScanContextIndex scan_context_index_;
  LoopClosureVerifier loop_closure_verifier_;
  Se2PoseGraphBackend pose_graph_backend_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string input_topic_;
  std::string odom_topic_;
  std::string base_frame_;
  std::string local_frame_;
  std::string map_frame_;
  double maximum_odom_age_{0.05};
  bool force_planar_motion_{true};
  double keyframe_translation_{0.25};
  double keyframe_rotation_{0.15};
  std::size_t maximum_odom_buffer_size_{200U};
  std::size_t maximum_pending_clouds_{5U};
  std::size_t maximum_loop_verification_candidates_{3U};
  std::size_t minimum_pose_graph_keyframe_interval_{30U};
  std::unique_ptr<MatchFailureRecovery> match_failure_recovery_;
  bool initialized_{false};
  Eigen::Isometry3d estimated_base_pose_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d last_odom_pose_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d last_keyframe_base_pose_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d last_global_keyframe_base_pose_{
    Eigen::Isometry3d::Identity()};
  bool has_last_global_keyframe_{false};
  double global_accumulated_distance_{0.0};
  std::vector<ScanContextCandidate> last_scan_context_candidates_;
  std::vector<LoopClosureVerificationResult> last_loop_verification_results_;
  std::size_t scan_context_index_size_{0U};
  std::deque<GlobalKeyframe> pending_loop_keyframes_;
  std::vector<Se2LoopConstraint> committed_loop_constraints_;
  std::vector<Se2LoopConstraint> pending_loop_constraints_;
  std::optional<std::future<LoopClosureTaskResult>> loop_closure_future_;
  std::optional<std::future<PoseGraphResult>> pose_graph_future_;
  Eigen::Isometry3d map_from_local_{Eigen::Isometry3d::Identity()};
  std::size_t pose_graph_commit_count_{0U};
  std::size_t pose_graph_discard_count_{0U};
  std::size_t pose_graph_failure_count_{0U};
  std::size_t last_pose_graph_submission_keyframe_id_{0U};
  bool has_pose_graph_submission_{false};
  std::size_t loop_closure_failure_count_{0U};
  std::deque<nav_msgs::msg::Odometry> odom_buffer_;
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> pending_clouds_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    registered_scan_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
};

}  // namespace slam_robot_slam_3d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<slam_robot_slam_3d::ScanToMapOdometryNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("scan_to_map_odometry_3d"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
