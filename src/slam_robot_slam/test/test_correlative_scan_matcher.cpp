#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

#include "gtest/gtest.h"
#include "slam_robot_slam/correlative_scan_matcher.hpp"

namespace
{

std::vector<slam_robot_slam::Point2D> makeRoomCorner()
{
  std::vector<slam_robot_slam::Point2D> points;
  for (int index = 0; index <= 80; ++index) {
    points.push_back(
      slam_robot_slam::Point2D{
        3.0F,
        static_cast<float>(-2.0 + 0.05 * index)});
  }
  for (int index = 1; index <= 100; ++index) {
    points.push_back(
      slam_robot_slam::Point2D{
        static_cast<float>(3.0 - 0.05 * index),
        2.0F});
  }
  for (int index = 1; index <= 50; ++index) {
    points.push_back(
      slam_robot_slam::Point2D{
        -2.0F,
        static_cast<float>(2.0 - 0.05 * index)});
  }
  return points;
}

std::vector<slam_robot_slam::Point2D> makeUniformParallelCorridor(
  const double minimum_x,
  const double maximum_x)
{
  std::vector<slam_robot_slam::Point2D> points;
  const int point_count = static_cast<int>(
    std::lround((maximum_x - minimum_x) / 0.02));
  points.reserve(static_cast<std::size_t>(2 * (point_count + 1)));
  for (int index = 0; index <= point_count; ++index) {
    const float x = static_cast<float>(minimum_x + 0.02 * index);
    points.push_back(slam_robot_slam::Point2D{x, -1.0F});
    points.push_back(slam_robot_slam::Point2D{x, 1.0F});
  }
  return points;
}

std::vector<slam_robot_slam::ScanPoint2D> makeAngularCorridorScan(
  const slam_robot_slam::Pose2D & sensor_pose,
  const double noise_stddev,
  const std::uint32_t noise_seed)
{
  constexpr std::size_t kSamples = 720U;
  constexpr std::size_t kPointStride = 2U;
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kMinimumRange = 0.12;
  constexpr double kMaximumRange = 12.0;
  constexpr double kHalfWidth = 1.0;
  std::mt19937 generator(noise_seed);
  std::normal_distribution<double> noise(0.0, noise_stddev);
  std::vector<slam_robot_slam::ScanPoint2D> points;
  points.reserve(kSamples / kPointStride);
  for (std::size_t index = 0U; index < kSamples; index += kPointStride) {
    const double angle =
      -kPi + 2.0 * kPi * static_cast<double>(index) /
      static_cast<double>(kSamples);
    const double world_angle = sensor_pose.yaw + angle;
    const double direction_y = std::sin(world_angle);
    if (std::abs(direction_y) < 1.0e-12) {
      continue;
    }
    const double wall_y = direction_y > 0.0 ? kHalfWidth : -kHalfWidth;
    const double ideal_range = (wall_y - sensor_pose.y) / direction_y;
    if (ideal_range < kMinimumRange || ideal_range > kMaximumRange) {
      continue;
    }
    const double measured_range = ideal_range + noise(generator);
    if (measured_range < kMinimumRange || measured_range > kMaximumRange) {
      continue;
    }
    points.push_back(
      slam_robot_slam::ScanPoint2D{
        slam_robot_slam::Point2D{
          static_cast<float>(measured_range * std::cos(angle)),
          static_cast<float>(measured_range * std::sin(angle))},
        index,
        static_cast<float>(measured_range)});
  }
  return points;
}

std::vector<slam_robot_slam::Point2D> makeAngularCorridorSubmap(
  const std::size_t keyframe_count,
  const double noise_stddev,
  const std::uint32_t noise_seed)
{
  std::vector<slam_robot_slam::Point2D> points;
  for (std::size_t index = 0U; index < keyframe_count; ++index) {
    const double x =
      -0.05 * static_cast<double>(keyframe_count - index - 1U);
    const slam_robot_slam::Pose2D keyframe_pose{x, 0.0, 0.0};
    const auto scan = makeAngularCorridorScan(
      keyframe_pose,
      noise_stddev,
      noise_seed + static_cast<std::uint32_t>(index));
    points.reserve(points.size() + scan.size());
    for (const auto & point : scan) {
      points.push_back(
        slam_robot_slam::transformPoint(keyframe_pose, point.point));
    }
  }
  return points;
}

slam_robot_slam::CorrelativeScanMatcherParameters runtimeMatcherParameters()
{
  slam_robot_slam::CorrelativeScanMatcherParameters parameters;
  parameters.grid_resolution = 0.02;
  parameters.smear_deviation = 0.10;
  parameters.linear_search_window = 0.15;
  parameters.angular_search_window = 0.20;
  parameters.coarse_linear_resolution = 0.04;
  parameters.coarse_angular_resolution = 0.04;
  parameters.fine_linear_window = 0.02;
  parameters.fine_angular_window = 0.02;
  parameters.fine_linear_resolution = 0.005;
  parameters.fine_angular_resolution = 0.005;
  parameters.translation_penalty_weight = 0.10;
  parameters.rotation_penalty_weight = 0.10;
  parameters.degeneracy_handling_enabled = true;
  parameters.weak_direction_correction_scale = 0.0;
  parameters.minimum_score = 0.35;
  parameters.minimum_support_fraction = 0.25;
  parameters.minimum_matched_points = 40U;
  return parameters;
}

TEST(CorrelativeScanMatcher, RecoversPoseAgainstLocalMap)
{
  const auto reference = makeRoomCorner();
  const slam_robot_slam::Pose2D expected{0.10, -0.06, 0.04};
  const auto current_from_reference =
    slam_robot_slam::inversePose(expected);
  std::vector<slam_robot_slam::Point2D> current;
  current.reserve(reference.size());
  for (const auto & point : reference) {
    current.push_back(
      slam_robot_slam::transformPoint(current_from_reference, point));
  }

  slam_robot_slam::CorrelativeScanMatcherParameters parameters;
  parameters.grid_resolution = 0.02;
  parameters.smear_deviation = 0.04;
  parameters.coarse_linear_resolution = 0.02;
  parameters.coarse_angular_resolution = 0.02;
  parameters.fine_linear_window = 0.02;
  parameters.fine_angular_window = 0.02;
  parameters.fine_linear_resolution = 0.005;
  parameters.fine_angular_resolution = 0.005;
  parameters.minimum_score = 0.50;
  parameters.minimum_matched_points = 80U;
  const auto result = slam_robot_slam::matchCorrelative(
    reference,
    current,
    slam_robot_slam::Pose2D{0.05, -0.02, 0.02},
    parameters);

  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.pose.x, expected.x, 0.011);
  EXPECT_NEAR(result.pose.y, expected.y, 0.011);
  EXPECT_NEAR(result.pose.yaw, expected.yaw, 0.011);
  EXPECT_GT(result.score, 0.80);
  EXPECT_GT(result.support_fraction, 0.90);
  EXPECT_GT(result.evaluated_candidates, 1000U);
}

TEST(CorrelativeScanMatcher, RetainsPredictionAlongIdealUniformCorridor)
{
  const auto reference = makeUniformParallelCorridor(-20.0, 20.0);
  const auto visible_world_points = makeUniformParallelCorridor(-5.0, 5.0);
  const slam_robot_slam::Pose2D actual_pose{0.10, 0.06, 0.02};
  const auto current_from_world =
    slam_robot_slam::inversePose(actual_pose);
  std::vector<slam_robot_slam::Point2D> current;
  current.reserve(visible_world_points.size());
  for (const auto & point : visible_world_points) {
    current.push_back(
      slam_robot_slam::transformPoint(current_from_world, point));
  }

  slam_robot_slam::CorrelativeScanMatcherParameters parameters;
  parameters.grid_resolution = 0.02;
  parameters.smear_deviation = 0.04;
  parameters.coarse_linear_resolution = 0.02;
  parameters.coarse_angular_resolution = 0.02;
  parameters.fine_linear_window = 0.02;
  parameters.fine_angular_window = 0.02;
  parameters.fine_linear_resolution = 0.005;
  parameters.fine_angular_resolution = 0.005;
  parameters.minimum_score = 0.50;
  parameters.minimum_matched_points = 100U;
  parameters.degeneracy_handling_enabled = true;
  parameters.weak_direction_correction_scale = 0.0;
  const slam_robot_slam::TranslationObservability observability{
    1U, 0.0, 100.0, 0.0,
    slam_robot_slam::Point2D{1.0F, 0.0F}, 100U, 0.0};
  const slam_robot_slam::Pose2D predicted_pose{0.0, 0.02, 0.01};
  const auto result = slam_robot_slam::matchCorrelative(
    reference,
    current,
    predicted_pose,
    parameters,
    &observability);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.translation_observable_rank, 1U);
  EXPECT_LT(result.translation_information_ratio, 0.05);
  EXPECT_GT(std::abs(result.weak_translation_direction.x), 0.95F);
  EXPECT_NEAR(result.pose.x, predicted_pose.x, 0.005);
  EXPECT_NEAR(result.pose.y, actual_pose.y, 0.011);
  EXPECT_NEAR(result.pose.yaw, actual_pose.yaw, 0.011);
}

TEST(CorrelativeScanMatcher, GeometryIsStableAcrossDepthAndNoise)
{
  constexpr std::array<std::size_t, 3U> kKeyframeCounts{1U, 5U, 20U};
  constexpr std::array<double, 4U> kNoiseStddevs{0.0, 0.0025, 0.005, 0.01};
  const slam_robot_slam::Pose2D actual_pose{0.10, 0.0, 0.0};
  std::size_t rank_one_cases = 0U;
  double maximum_longitudinal_error = 0.0;
  slam_robot_slam::TranslationObservabilityParameters observability_parameters;
  observability_parameters.beam_index_step = 2U;
  observability_parameters.normal_half_window = 3U;
  observability_parameters.maximum_neighbor_distance_base = 0.05;
  observability_parameters.maximum_neighbor_distance_ratio = 0.05;
  observability_parameters.minimum_normal_count = 20U;
  observability_parameters.minimum_effective_normal_count = 5.0;
  observability_parameters.minimum_information_ratio = 0.05;
  for (const std::size_t keyframe_count : kKeyframeCounts) {
    for (std::size_t noise_index = 0U;
      noise_index < kNoiseStddevs.size(); ++noise_index)
    {
      const double noise_stddev = kNoiseStddevs[noise_index];
      const std::uint32_t seed =
        100U + static_cast<std::uint32_t>(10U * keyframe_count + noise_index);
      const auto reference = makeAngularCorridorSubmap(
        keyframe_count, noise_stddev, seed);
      const auto current_scan = makeAngularCorridorScan(
        actual_pose, noise_stddev, seed + 1000U);
      std::vector<slam_robot_slam::Point2D> current;
      current.reserve(current_scan.size());
      for (const auto & point : current_scan) {
        current.push_back(point.point);
      }
      const auto observability =
        slam_robot_slam::estimateNormalTranslationObservability(
        current_scan, observability_parameters);
      const auto result = slam_robot_slam::matchCorrelative(
        reference,
        current,
        actual_pose,
        runtimeMatcherParameters(),
        &observability);

      SCOPED_TRACE(
        ::testing::Message() << "keyframes=" << keyframe_count <<
          " noise=" << noise_stddev);
      ASSERT_TRUE(result.success);
      EXPECT_GT(result.score, 0.90);
      EXPECT_GT(result.support_fraction, 0.95);
      EXPECT_GT(std::abs(result.weak_translation_direction.x), 0.95F);
      EXPECT_EQ(result.translation_observable_rank, 1U);
      EXPECT_GT(result.translation_normal_count, 100U);
      maximum_longitudinal_error = std::max(
        maximum_longitudinal_error,
        std::abs(result.pose.x - actual_pose.x));
      if (result.translation_observable_rank == 1U) {
        ++rank_one_cases;
      }
    }
  }

  EXPECT_EQ(rank_one_cases, 12U);
  EXPECT_LE(maximum_longitudinal_error, 0.011);
}

TEST(CorrelativeScanMatcher, PreservesTranslationCorrectionInCorner)
{
  const auto reference = makeRoomCorner();
  const slam_robot_slam::Pose2D expected{0.10, -0.06, 0.04};
  const auto current_from_reference =
    slam_robot_slam::inversePose(expected);
  std::vector<slam_robot_slam::Point2D> current;
  current.reserve(reference.size());
  for (const auto & point : reference) {
    current.push_back(
      slam_robot_slam::transformPoint(current_from_reference, point));
  }

  slam_robot_slam::CorrelativeScanMatcherParameters parameters;
  parameters.grid_resolution = 0.02;
  parameters.smear_deviation = 0.04;
  parameters.coarse_linear_resolution = 0.02;
  parameters.coarse_angular_resolution = 0.02;
  parameters.fine_linear_window = 0.02;
  parameters.fine_angular_window = 0.02;
  parameters.fine_linear_resolution = 0.005;
  parameters.fine_angular_resolution = 0.005;
  parameters.minimum_score = 0.50;
  parameters.minimum_matched_points = 80U;
  parameters.degeneracy_handling_enabled = true;
  const slam_robot_slam::TranslationObservability observability{
    2U, 50.0, 50.0, 1.0,
    slam_robot_slam::Point2D{1.0F, 0.0F}, 100U, 1.0};
  const auto result = slam_robot_slam::matchCorrelative(
    reference,
    current,
    slam_robot_slam::Pose2D{0.05, -0.02, 0.02},
    parameters,
    &observability);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.translation_observable_rank, 2U);
  EXPECT_NEAR(result.pose.x, expected.x, 0.011);
  EXPECT_NEAR(result.pose.y, expected.y, 0.011);
}

TEST(CorrelativeScanMatcher, RankTwoDegeneracyHandlingIsAnExactNoOp)
{
  const auto reference = makeRoomCorner();
  const slam_robot_slam::Pose2D expected{0.10, -0.06, 0.04};
  const auto current_from_reference =
    slam_robot_slam::inversePose(expected);
  std::vector<slam_robot_slam::Point2D> current;
  current.reserve(reference.size());
  for (const auto & point : reference) {
    current.push_back(
      slam_robot_slam::transformPoint(current_from_reference, point));
  }

  slam_robot_slam::CorrelativeScanMatcherParameters disabled_parameters;
  disabled_parameters.grid_resolution = 0.02;
  disabled_parameters.smear_deviation = 0.04;
  disabled_parameters.coarse_linear_resolution = 0.02;
  disabled_parameters.coarse_angular_resolution = 0.02;
  disabled_parameters.fine_linear_window = 0.02;
  disabled_parameters.fine_angular_window = 0.02;
  disabled_parameters.fine_linear_resolution = 0.005;
  disabled_parameters.fine_angular_resolution = 0.005;
  disabled_parameters.minimum_score = 0.50;
  disabled_parameters.minimum_matched_points = 80U;
  const slam_robot_slam::Pose2D predicted{0.05, -0.02, 0.02};
  const auto disabled_result = slam_robot_slam::matchCorrelative(
    reference, current, predicted, disabled_parameters);

  auto enabled_parameters = disabled_parameters;
  enabled_parameters.degeneracy_handling_enabled = true;
  const slam_robot_slam::TranslationObservability observability{
    2U, 50.0, 50.0, 1.0,
    slam_robot_slam::Point2D{1.0F, 0.0F}, 100U, 1.0};
  const auto enabled_result = slam_robot_slam::matchCorrelative(
    reference,
    current,
    predicted,
    enabled_parameters,
    &observability);

  ASSERT_TRUE(disabled_result.success);
  ASSERT_TRUE(enabled_result.success);
  EXPECT_DOUBLE_EQ(enabled_result.pose.x, disabled_result.pose.x);
  EXPECT_DOUBLE_EQ(enabled_result.pose.y, disabled_result.pose.y);
  EXPECT_DOUBLE_EQ(enabled_result.pose.yaw, disabled_result.pose.yaw);
  EXPECT_DOUBLE_EQ(enabled_result.score, disabled_result.score);
  EXPECT_EQ(
    enabled_result.matched_points,
    disabled_result.matched_points);
  EXPECT_EQ(
    enabled_result.evaluated_candidates,
    disabled_result.evaluated_candidates);
}

TEST(CorrelativeScanMatcher, RejectsScanWithoutOverlap)
{
  const auto reference = makeRoomCorner();
  std::vector<slam_robot_slam::Point2D> current{
    {20.0F, 20.0F},
    {20.1F, 20.0F},
    {20.2F, 20.0F}};

  slam_robot_slam::CorrelativeScanMatcherParameters parameters;
  parameters.minimum_matched_points = 3U;
  const auto result = slam_robot_slam::matchCorrelative(
    reference,
    current,
    slam_robot_slam::Pose2D{},
    parameters);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.matched_points, 0U);
}

TEST(CorrelativeScanMatcher, ScoresSupportedOverlapSeparately)
{
  const auto reference = makeRoomCorner();
  std::vector<slam_robot_slam::Point2D> current = reference;
  for (int index = 0; index < 200; ++index) {
    current.push_back(
      slam_robot_slam::Point2D{
        20.0F + static_cast<float>(index) * 0.01F,
        20.0F});
  }

  slam_robot_slam::CorrelativeScanMatcherParameters parameters;
  parameters.minimum_score = 0.8;
  parameters.minimum_support_fraction = 0.4;
  parameters.minimum_matched_points = 100U;
  const auto result = slam_robot_slam::matchCorrelative(
    reference,
    current,
    slam_robot_slam::Pose2D{},
    parameters);

  EXPECT_TRUE(result.success);
  EXPECT_GT(result.score, 0.9);
  EXPECT_GT(result.support_fraction, 0.4);
  EXPECT_LT(result.support_fraction, 0.6);
}

TEST(CorrelativeScanMatcher, RejectsInvalidParameters)
{
  slam_robot_slam::CorrelativeScanMatcherParameters parameters;
  parameters.grid_resolution = 0.0;

  EXPECT_THROW(
    slam_robot_slam::matchCorrelative(
      makeRoomCorner(),
      makeRoomCorner(),
      slam_robot_slam::Pose2D{},
      parameters),
    std::invalid_argument);

  parameters = slam_robot_slam::CorrelativeScanMatcherParameters{};
  parameters.minimum_score =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    slam_robot_slam::validateCorrelativeScanMatcherParameters(parameters),
    std::invalid_argument);

  parameters = slam_robot_slam::CorrelativeScanMatcherParameters{};
  parameters.minimum_support_fraction = 0.0;
  EXPECT_THROW(
    slam_robot_slam::validateCorrelativeScanMatcherParameters(parameters),
    std::invalid_argument);

  parameters = slam_robot_slam::CorrelativeScanMatcherParameters{};
  parameters.degeneracy_handling_enabled = true;
  EXPECT_THROW(
    slam_robot_slam::matchCorrelative(
      makeRoomCorner(),
      makeRoomCorner(),
      slam_robot_slam::Pose2D{},
      parameters),
    std::invalid_argument);
}

TEST(CorrelativeScanMatcher, RejectsNonFinitePoints)
{
  auto current = makeRoomCorner();
  current.front().x = std::numeric_limits<float>::quiet_NaN();

  EXPECT_THROW(
    slam_robot_slam::matchCorrelative(
      makeRoomCorner(),
      current,
      slam_robot_slam::Pose2D{},
      slam_robot_slam::CorrelativeScanMatcherParameters{}),
    std::invalid_argument);
}

}  // namespace
