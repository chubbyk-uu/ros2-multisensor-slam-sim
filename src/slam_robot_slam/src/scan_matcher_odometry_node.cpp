#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

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
    const double sequential_translation_stddev =
      declare_parameter<double>(
      "pose_graph.sequential_translation_stddev", 0.05);
    const double sequential_rotation_stddev =
      declare_parameter<double>(
      "pose_graph.sequential_rotation_stddev", 0.05);

    OccupancyGridMapParameters map_parameters;
    map_parameters.resolution =
      declare_parameter<double>("map.resolution", 0.05);
    map_parameters.hit_probability =
      declare_parameter<double>("map.hit_probability", 0.70);
    map_parameters.miss_probability =
      declare_parameter<double>("map.miss_probability", 0.40);
    map_parameters.minimum_probability =
      declare_parameter<double>("map.minimum_probability", 0.12);
    map_parameters.maximum_probability =
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

    if (minimum_range_ < 0.0 || maximum_range_ <= minimum_range_ ||
      point_stride < 1 || maximum_odom_age_ <= 0.0 ||
      minimum_translation_for_update_ < 0.0 ||
      minimum_rotation_for_update_ < 0.0 ||
      maximum_local_keyframes < 1 || maximum_path_poses_ < 1 ||
      minimum_matched_points < 1 || map_ray_stride < 1 ||
      map_publish_period <= 0.0 || map_padding_cells < 0 ||
      map_padding_cells > std::numeric_limits<int>::max() ||
      sequential_translation_stddev <= 0.0 ||
      sequential_rotation_stddev <= 0.0)
    {
      throw std::invalid_argument("Invalid scan matcher node parameters");
    }
    point_stride_ = static_cast<std::size_t>(point_stride);
    maximum_local_keyframes_ =
      static_cast<std::size_t>(maximum_local_keyframes);
    matcher_parameters_.minimum_matched_points =
      static_cast<std::size_t>(minimum_matched_points);
    map_ray_stride_ = static_cast<std::size_t>(map_ray_stride);
    map_parameters.padding_cells = static_cast<int>(map_padding_cells);
    sequential_translation_weight_ =
      1.0 / sequential_translation_stddev;
    sequential_rotation_weight_ =
      1.0 / sequential_rotation_stddev;
    occupancy_grid_map_ =
      std::make_unique<OccupancyGridMap>(map_parameters);

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
      map_parameters.resolution);
  }

private:
  struct OdomSample
  {
    int64_t stamp_nanoseconds;
    Pose2D pose;
  };

  struct Keyframe
  {
    Pose2D pose;
    std::vector<Point2D> points;
  };

  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odometry)
  {
    const auto & position = odometry->pose.pose.position;
    odom_samples_.push_back(
      OdomSample{
        stampToNanoseconds(odometry->header.stamp),
        Pose2D{
          position.x,
          position.y,
          quaternionToYaw(odometry->pose.pose.orientation)}});

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
      addKeyframe(scan->header.stamp, estimated_pose_, base_points);
      integrateMap(*scan, base_from_laser);
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
      addKeyframe(scan->header.stamp, estimated_pose_, base_points);
      integrateMap(*scan, base_from_laser);
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
    const std::vector<Point2D> & points)
  {
    keyframes_.push_back(Keyframe{pose, points});

    const std::size_t node_id = pose_graph_.addNode(pose);
    if (node_id > 0U) {
      const Pose2D & previous_pose =
        keyframes_[node_id - 1U].pose;
      pose_graph_.addConstraint(
        PoseGraphConstraint{
          node_id - 1U,
          node_id,
          relativePose(previous_pose, pose),
          sequential_translation_weight_,
          sequential_rotation_weight_,
          PoseGraphConstraintType::kSequential});
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

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Pose graph contains %zu nodes and %zu sequential constraints",
      pose_graph_.nodes().size(),
      pose_graph_.constraints().size());
  }

  std::vector<Point2D> buildLocalReferencePoints() const
  {
    std::size_t total_points = 0U;
    const std::size_t first_keyframe =
      keyframes_.size() > maximum_local_keyframes_ ?
      keyframes_.size() - maximum_local_keyframes_ : 0U;
    for (std::size_t index = first_keyframe;
      index < keyframes_.size(); ++index)
    {
      total_points += keyframes_[index].points.size();
    }

    std::vector<Point2D> reference_points;
    reference_points.reserve(total_points);
    for (std::size_t index = first_keyframe;
      index < keyframes_.size(); ++index)
    {
      const auto & keyframe = keyframes_[index];
      std::transform(
        keyframe.points.begin(),
        keyframe.points.end(),
        std::back_inserter(reference_points),
        [&keyframe](const Point2D & point) {
          return transformPoint(keyframe.pose, point);
        });
    }
    return reference_points;
  }

  void integrateMap(
    const sensor_msgs::msg::LaserScan & scan,
    const Pose2D & base_from_laser)
  {
    const double effective_minimum =
      std::max(minimum_range_, static_cast<double>(scan.range_min));
    const double effective_maximum =
      std::min(maximum_range_, static_cast<double>(scan.range_max));
    if (effective_minimum >= effective_maximum) {
      return;
    }

    const Point2D sensor_origin = transformPoint(
      estimated_pose_,
      Point2D{
        static_cast<float>(base_from_laser.x),
        static_cast<float>(base_from_laser.y)});
    std::size_t integrated_rays = 0U;
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
      const Point2D map_endpoint =
        transformPoint(estimated_pose_, base_endpoint);
      occupancy_grid_map_->updateRay(
        sensor_origin, map_endpoint, endpoint_is_hit);
      ++integrated_rays;
    }

    if (integrated_rays > 0U) {
      latest_map_stamp_ = scan.header.stamp;
      map_dirty_ = true;
    }
  }

  void publishMapIfDirty()
  {
    if (!map_dirty_) {
      return;
    }

    const auto snapshot = occupancy_grid_map_->snapshot();
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
  double sequential_translation_weight_;
  double sequential_rotation_weight_;
  CorrelativeScanMatcherParameters matcher_parameters_;
  std::unique_ptr<OccupancyGridMap> occupancy_grid_map_;
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
};

}  // namespace slam_robot_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<slam_robot_slam::ScanMatcherOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
