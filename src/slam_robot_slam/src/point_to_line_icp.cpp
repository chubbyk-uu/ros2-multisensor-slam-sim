#include "slam_robot_slam/point_to_line_icp.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace slam_robot_slam
{
namespace
{

struct ReferenceLinePoint
{
  Point2D point;
  Point2D normal;
};

using Matrix3 = std::array<std::array<double, 3>, 3>;
using Vector3 = std::array<double, 3>;

std::vector<ReferenceLinePoint> estimateReferenceNormals(
  const std::vector<Point2D> & points,
  const double maximum_neighbor_distance)
{
  std::vector<ReferenceLinePoint> reference;
  if (points.size() < 3U) {
    return reference;
  }

  const double maximum_squared =
    maximum_neighbor_distance * maximum_neighbor_distance;
  reference.reserve(points.size() - 2U);

  for (std::size_t index = 1U; index + 1U < points.size(); ++index) {
    const auto & previous = points[index - 1U];
    const auto & point = points[index];
    const auto & next = points[index + 1U];

    const double previous_dx =
      static_cast<double>(point.x) - static_cast<double>(previous.x);
    const double previous_dy =
      static_cast<double>(point.y) - static_cast<double>(previous.y);
    const double next_dx =
      static_cast<double>(next.x) - static_cast<double>(point.x);
    const double next_dy =
      static_cast<double>(next.y) - static_cast<double>(point.y);
    if (previous_dx * previous_dx + previous_dy * previous_dy >
      maximum_squared ||
      next_dx * next_dx + next_dy * next_dy > maximum_squared)
    {
      continue;
    }

    const double tangent_x =
      static_cast<double>(next.x) - static_cast<double>(previous.x);
    const double tangent_y =
      static_cast<double>(next.y) - static_cast<double>(previous.y);
    const double tangent_norm = std::hypot(tangent_x, tangent_y);
    if (tangent_norm < 1.0e-6) {
      continue;
    }

    reference.push_back(
      ReferenceLinePoint{
          point,
          Point2D{
            static_cast<float>(-tangent_y / tangent_norm),
            static_cast<float>(tangent_x / tangent_norm)}});
  }

  return reference;
}

bool solveLinearSystem(
  Matrix3 matrix,
  Vector3 right_hand_side,
  Vector3 & solution)
{
  for (std::size_t pivot = 0U; pivot < 3U; ++pivot) {
    std::size_t best_row = pivot;
    double best_value = std::abs(matrix[pivot][pivot]);
    for (std::size_t row = pivot + 1U; row < 3U; ++row) {
      const double candidate = std::abs(matrix[row][pivot]);
      if (candidate > best_value) {
        best_row = row;
        best_value = candidate;
      }
    }
    if (best_value < 1.0e-12) {
      return false;
    }

    if (best_row != pivot) {
      std::swap(matrix[pivot], matrix[best_row]);
      std::swap(right_hand_side[pivot], right_hand_side[best_row]);
    }

    for (std::size_t row = pivot + 1U; row < 3U; ++row) {
      const double factor = matrix[row][pivot] / matrix[pivot][pivot];
      for (std::size_t column = pivot; column < 3U; ++column) {
        matrix[row][column] -= factor * matrix[pivot][column];
      }
      right_hand_side[row] -= factor * right_hand_side[pivot];
    }
  }

  for (int row = 2; row >= 0; --row) {
    double value = right_hand_side[static_cast<std::size_t>(row)];
    for (std::size_t column = static_cast<std::size_t>(row) + 1U;
      column < 3U; ++column)
    {
      value -= matrix[static_cast<std::size_t>(row)][column] * solution[column];
    }
    solution[static_cast<std::size_t>(row)] =
      value /
      matrix[static_cast<std::size_t>(row)][static_cast<std::size_t>(row)];
  }

  return std::all_of(
    solution.begin(), solution.end(),
    [](const double value) {return std::isfinite(value);});
}

void validateParameters(const IcpParameters & parameters)
{
  if (parameters.maximum_iterations == 0U ||
    parameters.minimum_correspondences < 3U ||
    parameters.maximum_correspondence_distance <= 0.0 ||
    parameters.maximum_neighbor_distance <= 0.0 ||
    parameters.huber_scale <= 0.0 ||
    parameters.translation_convergence <= 0.0 ||
    parameters.rotation_convergence <= 0.0 ||
    parameters.maximum_mean_error <= 0.0 ||
    parameters.translation_prior_weight < 0.0 ||
    parameters.rotation_prior_weight < 0.0 ||
    parameters.damping < 0.0)
  {
    throw std::invalid_argument("Invalid point-to-line ICP parameters");
  }
}

}  // namespace

IcpResult matchPointToLineIcp(
  const std::vector<Point2D> & reference_points,
  const std::vector<Point2D> & current_points,
  const Pose2D & initial_pose,
  const IcpParameters & parameters)
{
  validateParameters(parameters);

  IcpResult result;
  result.pose = initial_pose;
  const auto reference = estimateReferenceNormals(
    reference_points, parameters.maximum_neighbor_distance);
  if (reference.size() < parameters.minimum_correspondences ||
    current_points.size() < parameters.minimum_correspondences)
  {
    return result;
  }

  const double maximum_correspondence_squared =
    parameters.maximum_correspondence_distance *
    parameters.maximum_correspondence_distance;
  bool solved_iteration = false;

  for (std::size_t iteration = 0U;
    iteration < parameters.maximum_iterations; ++iteration)
  {
    Matrix3 hessian{};
    Vector3 gradient{};
    std::size_t correspondences = 0U;
    double absolute_error_sum = 0.0;

    const double cosine = std::cos(result.pose.yaw);
    const double sine = std::sin(result.pose.yaw);

    for (const auto & source : current_points) {
      const Point2D transformed = transformPoint(result.pose, source);
      const ReferenceLinePoint * nearest = nullptr;
      double nearest_squared = std::numeric_limits<double>::max();

      for (const auto & candidate : reference) {
        const double dx =
          static_cast<double>(transformed.x) -
          static_cast<double>(candidate.point.x);
        const double dy =
          static_cast<double>(transformed.y) -
          static_cast<double>(candidate.point.y);
        const double squared_distance = dx * dx + dy * dy;
        if (squared_distance < nearest_squared) {
          nearest_squared = squared_distance;
          nearest = &candidate;
        }
      }

      if (nearest == nullptr ||
        nearest_squared > maximum_correspondence_squared)
      {
        continue;
      }

      const double normal_x = static_cast<double>(nearest->normal.x);
      const double normal_y = static_cast<double>(nearest->normal.y);
      const double residual =
        normal_x *
        (static_cast<double>(transformed.x) -
        static_cast<double>(nearest->point.x)) +
        normal_y *
        (static_cast<double>(transformed.y) -
        static_cast<double>(nearest->point.y));
      const double absolute_residual = std::abs(residual);
      const double weight = absolute_residual <= parameters.huber_scale ?
        1.0 : parameters.huber_scale / absolute_residual;

      const double derivative_x =
        -sine * static_cast<double>(source.x) -
        cosine * static_cast<double>(source.y);
      const double derivative_y =
        cosine * static_cast<double>(source.x) -
        sine * static_cast<double>(source.y);
      const Vector3 jacobian{
        normal_x,
        normal_y,
        normal_x * derivative_x + normal_y * derivative_y};

      for (std::size_t row = 0U; row < 3U; ++row) {
        gradient[row] += weight * jacobian[row] * residual;
        for (std::size_t column = 0U; column < 3U; ++column) {
          hessian[row][column] +=
            weight * jacobian[row] * jacobian[column];
        }
      }
      ++correspondences;
      absolute_error_sum += absolute_residual;
    }

    result.iterations = iteration + 1U;
    result.correspondences = correspondences;
    if (correspondences < parameters.minimum_correspondences) {
      return result;
    }
    result.mean_absolute_error =
      absolute_error_sum / static_cast<double>(correspondences);

    const Vector3 prior_error{
      result.pose.x - initial_pose.x,
      result.pose.y - initial_pose.y,
      normalizeAngle(result.pose.yaw - initial_pose.yaw)};
    hessian[0][0] += parameters.translation_prior_weight;
    hessian[1][1] += parameters.translation_prior_weight;
    hessian[2][2] += parameters.rotation_prior_weight;
    gradient[0] += parameters.translation_prior_weight * prior_error[0];
    gradient[1] += parameters.translation_prior_weight * prior_error[1];
    gradient[2] += parameters.rotation_prior_weight * prior_error[2];

    for (std::size_t diagonal = 0U; diagonal < 3U; ++diagonal) {
      hessian[diagonal][diagonal] += parameters.damping;
    }
    Vector3 increment{};
    const Vector3 right_hand_side{
      -gradient[0], -gradient[1], -gradient[2]};
    if (!solveLinearSystem(hessian, right_hand_side, increment)) {
      return result;
    }

    result.pose.x += increment[0];
    result.pose.y += increment[1];
    result.pose.yaw = normalizeAngle(result.pose.yaw + increment[2]);
    solved_iteration = true;

    if (std::hypot(increment[0], increment[1]) <
      parameters.translation_convergence &&
      std::abs(increment[2]) < parameters.rotation_convergence)
    {
      result.converged = true;
      break;
    }
  }

  result.success =
    solved_iteration &&
    result.mean_absolute_error <= parameters.maximum_mean_error;
  return result;
}

}  // namespace slam_robot_slam
