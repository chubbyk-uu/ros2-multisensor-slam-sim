#ifndef SLAM_ROBOT_SLAM_3D__GLOBAL_KEYFRAME_MAP_HPP_
#define SLAM_ROBOT_SLAM_3D__GLOBAL_KEYFRAME_MAP_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <shared_mutex>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/time.hpp>

namespace slam_robot_slam_3d
{

// A global keyframe owns no mutable map state. The scan remains in its sensor
// frame, while base_to_sensor makes later map replay independent of the live
// TF buffer. The registration scan is deliberately coarse and feeds all
// registration, loop-closure and 3D-map consumers. The occupancy scan retains
// the mapping-grid resolution and is used only for 2D ray tracing.
struct GlobalKeyframe
{
  std::size_t id{0U};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> registration_scan;
  std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> occupancy_scan;
  Eigen::Isometry3d front_end_base_pose{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d odom_base_pose{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d base_to_sensor{Eigen::Isometry3d::Identity()};
  Eigen::Matrix<double, 6, 6> pose_covariance{
    Eigen::Matrix<double, 6, 6>::Identity()};
  double accumulated_distance{0.0};
  bool match_accepted{false};
  bool translation_degenerate{false};
  bool planar_degenerate{false};
  bool yaw_degenerate{false};
  std::size_t correspondence_count{0U};
  double rmse{0.0};
};

class GlobalKeyframeMap
{
public:
  std::size_t add(GlobalKeyframe keyframe);
  std::vector<GlobalKeyframe> snapshot() const;
  void replace(std::vector<GlobalKeyframe> keyframes);
  std::size_t size() const;
  // Kept as the registration-point count for existing diagnostics.
  std::size_t pointCount() const;
  std::size_t occupancyPointCount() const;

private:
  static void validateKeyframe(const GlobalKeyframe & keyframe);

  mutable std::shared_mutex mutex_;
  std::vector<GlobalKeyframe> keyframes_;
  std::size_t point_count_{0U};
  std::size_t occupancy_point_count_{0U};
};

}  // namespace slam_robot_slam_3d

#endif  // SLAM_ROBOT_SLAM_3D__GLOBAL_KEYFRAME_MAP_HPP_
