#ifndef SLAM_ROBOT_SLAM__POINT_TO_LINE_ICP_HPP_
#define SLAM_ROBOT_SLAM__POINT_TO_LINE_ICP_HPP_

#include <cstddef>
#include <vector>

#include "slam_robot_slam/pose2d.hpp"

namespace slam_robot_slam
{

struct IcpParameters
{
  std::size_t maximum_iterations{15U};
  std::size_t minimum_correspondences{40U};
  double maximum_correspondence_distance{0.30};
  double maximum_neighbor_distance{0.35};
  double huber_scale{0.05};
  double translation_convergence{1.0e-4};
  double rotation_convergence{1.0e-4};
  double maximum_mean_error{0.08};
  double translation_prior_weight{0.0};
  double rotation_prior_weight{0.0};
  double damping{1.0e-6};
};

struct IcpResult
{
  Pose2D pose;
  bool success{false};
  bool converged{false};
  std::size_t iterations{0U};
  std::size_t correspondences{0U};
  double mean_absolute_error{0.0};
};

IcpResult matchPointToLineIcp(
  const std::vector<Point2D> & reference_points,
  const std::vector<Point2D> & current_points,
  const Pose2D & initial_pose,
  const IcpParameters & parameters);

}  // namespace slam_robot_slam

#endif  // SLAM_ROBOT_SLAM__POINT_TO_LINE_ICP_HPP_
