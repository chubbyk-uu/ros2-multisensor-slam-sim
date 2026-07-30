#include "slam_robot_slam/pose2d.hpp"

#include <cmath>

namespace slam_robot_slam
{

double normalizeAngle(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 2.0 * kPi;
  while (angle > kPi) {
    angle -= kTwoPi;
  }
  while (angle < -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

Point2D transformPoint(const Pose2D & pose, const Point2D & point)
{
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  return Point2D{
    static_cast<float>(
      cosine * static_cast<double>(point.x) -
      sine * static_cast<double>(point.y) + pose.x),
    static_cast<float>(
      sine * static_cast<double>(point.x) +
      cosine * static_cast<double>(point.y) + pose.y)};
}

Pose2D composePoses(const Pose2D & first, const Pose2D & second)
{
  const double cosine = std::cos(first.yaw);
  const double sine = std::sin(first.yaw);
  return Pose2D{
    first.x + cosine * second.x - sine * second.y,
    first.y + sine * second.x + cosine * second.y,
    normalizeAngle(first.yaw + second.yaw)};
}

Pose2D inversePose(const Pose2D & pose)
{
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  return Pose2D{
    -cosine * pose.x - sine * pose.y,
    sine * pose.x - cosine * pose.y,
    normalizeAngle(-pose.yaw)};
}

Pose2D relativePose(const Pose2D & from, const Pose2D & to)
{
  return composePoses(inversePose(from), to);
}

}  // namespace slam_robot_slam
