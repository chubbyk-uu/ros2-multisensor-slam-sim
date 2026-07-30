#ifndef SLAM_ROBOT_SLAM__POSE2D_HPP_
#define SLAM_ROBOT_SLAM__POSE2D_HPP_

#include "slam_robot_slam/laser_scan_preprocessor.hpp"

namespace slam_robot_slam
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

bool isFinitePose(const Pose2D & pose);
double normalizeAngle(double angle);
Point2D transformPoint(const Pose2D & pose, const Point2D & point);
Pose2D composePoses(const Pose2D & first, const Pose2D & second);
Pose2D inversePose(const Pose2D & pose);
Pose2D relativePose(const Pose2D & from, const Pose2D & to);
Pose2D interpolatePoses(
  const Pose2D & first,
  const Pose2D & second,
  double ratio);

}  // namespace slam_robot_slam

#endif  // SLAM_ROBOT_SLAM__POSE2D_HPP_
