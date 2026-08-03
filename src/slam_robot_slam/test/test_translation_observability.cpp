#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "slam_robot_slam/translation_observability.hpp"

namespace
{

slam_robot_slam::ScanPoint2D makeScanPoint(
  const float x,
  const float y,
  const std::size_t beam_index)
{
  return slam_robot_slam::ScanPoint2D{
    slam_robot_slam::Point2D{x, y},
    beam_index,
    std::hypot(x, y)};
}

slam_robot_slam::TranslationObservabilityParameters testParameters()
{
  slam_robot_slam::TranslationObservabilityParameters parameters;
  parameters.beam_index_step = 1U;
  parameters.normal_half_window = 1U;
  parameters.maximum_neighbor_distance_base = 0.08;
  parameters.maximum_neighbor_distance_ratio = 0.0;
  parameters.minimum_normal_count = 4U;
  parameters.minimum_effective_normal_count = 1.0;
  parameters.minimum_information_ratio = 0.05;
  return parameters;
}

TEST(TranslationObservability, DetectsWeakDirectionFromParallelSurface)
{
  std::vector<slam_robot_slam::ScanPoint2D> points;
  for (std::size_t index = 0U; index <= 40U; ++index) {
    points.push_back(
      makeScanPoint(-1.0F + 0.05F * static_cast<float>(index), 1.0F, index));
  }

  const auto result =
    slam_robot_slam::estimateNormalTranslationObservability(
    points, testParameters());

  EXPECT_EQ(result.rank, 1U);
  EXPECT_EQ(result.normal_count, 39U);
  EXPECT_NEAR(result.minimum_information, 0.0, 1.0e-9);
  EXPECT_NEAR(result.maximum_information, 39.0, 1.0e-9);
  EXPECT_NEAR(result.information_ratio, 0.0, 1.0e-9);
  EXPECT_GT(std::abs(result.weak_direction.x), 0.999F);
}

TEST(TranslationObservability, PreservesRankTwoForCornerGeometry)
{
  std::vector<slam_robot_slam::ScanPoint2D> points;
  for (std::size_t index = 0U; index <= 40U; ++index) {
    points.push_back(
      makeScanPoint(-1.0F + 0.05F * static_cast<float>(index), 1.0F, index));
  }
  for (std::size_t index = 0U; index <= 40U; ++index) {
    points.push_back(
      makeScanPoint(
        1.0F,
        0.95F - 0.05F * static_cast<float>(index),
        41U + index));
  }

  const auto result =
    slam_robot_slam::estimateNormalTranslationObservability(
    points, testParameters());

  EXPECT_EQ(result.rank, 2U);
  EXPECT_GT(result.normal_count, 70U);
  EXPECT_GT(result.minimum_information, 30.0);
  EXPECT_GT(result.information_ratio, 0.7);
}

TEST(TranslationObservability, ScalesWeakCorrectionContinuouslyNearThreshold)
{
  std::vector<slam_robot_slam::ScanPoint2D> points;
  for (std::size_t index = 0U; index <= 40U; ++index) {
    points.push_back(
      makeScanPoint(-1.0F + 0.05F * static_cast<float>(index), 1.0F, index));
  }
  for (std::size_t index = 0U; index <= 10U; ++index) {
    points.push_back(
      makeScanPoint(
        1.0F,
        0.95F - 0.05F * static_cast<float>(index),
        41U + index));
  }
  auto parameters = testParameters();
  parameters.minimum_effective_normal_count = 0.1;
  parameters.minimum_information_ratio = 0.5;

  const auto result =
    slam_robot_slam::estimateNormalTranslationObservability(
    points, parameters);

  ASSERT_EQ(result.rank, 1U);
  EXPECT_GT(result.weak_direction_correction_scale, 0.0);
  EXPECT_LT(result.weak_direction_correction_scale, 1.0);
  EXPECT_NEAR(
    result.weak_direction_correction_scale,
    result.information_ratio / parameters.minimum_information_ratio,
    1.0e-9);
}

TEST(TranslationObservability, ScalesWeakCorrectionWhenAbsoluteInformationIsLow)
{
  std::vector<slam_robot_slam::ScanPoint2D> points;
  for (std::size_t index = 0U; index <= 40U; ++index) {
    points.push_back(
      makeScanPoint(-1.0F + 0.05F * static_cast<float>(index), 1.0F, index));
  }
  for (std::size_t index = 0U; index <= 10U; ++index) {
    points.push_back(
      makeScanPoint(
        1.0F,
        0.95F - 0.05F * static_cast<float>(index),
        41U + index));
  }
  auto parameters = testParameters();
  parameters.minimum_effective_normal_count = 20.0;
  parameters.minimum_information_ratio = 0.05;

  const auto result =
    slam_robot_slam::estimateNormalTranslationObservability(
    points, parameters);

  ASSERT_EQ(result.rank, 1U);
  ASSERT_GT(result.information_ratio, parameters.minimum_information_ratio);
  ASSERT_LT(
    result.minimum_information,
    parameters.minimum_effective_normal_count);
  EXPECT_GT(result.weak_direction_correction_scale, 0.0);
  EXPECT_LT(result.weak_direction_correction_scale, 1.0);
  EXPECT_NEAR(
    result.weak_direction_correction_scale,
    result.minimum_information / parameters.minimum_effective_normal_count,
    1.0e-9);
}

TEST(TranslationObservability, ReportsRankZeroWhenBothDirectionsAreWeak)
{
  std::vector<slam_robot_slam::ScanPoint2D> points;
  for (std::size_t index = 0U; index <= 10U; ++index) {
    points.push_back(
      makeScanPoint(-0.25F + 0.05F * static_cast<float>(index), 1.0F, index));
  }
  auto parameters = testParameters();
  parameters.minimum_effective_normal_count = 20.0;

  const auto result =
    slam_robot_slam::estimateNormalTranslationObservability(
    points, parameters);

  EXPECT_EQ(result.rank, 0U);
  EXPECT_LT(
    result.maximum_information,
    parameters.minimum_effective_normal_count);
  EXPECT_DOUBLE_EQ(result.weak_direction_correction_scale, 0.0);
}

TEST(TranslationObservability, RejectsBeamGapsAndSurfaceBreaks)
{
  std::vector<slam_robot_slam::ScanPoint2D> points{
    makeScanPoint(0.00F, 1.0F, 0U),
    makeScanPoint(0.05F, 1.0F, 1U),
    makeScanPoint(0.10F, 1.0F, 2U),
    makeScanPoint(0.15F, 1.0F, 3U),
    makeScanPoint(0.20F, 1.0F, 4U),
    makeScanPoint(2.00F, 1.0F, 10U),
    makeScanPoint(2.05F, 1.0F, 11U),
    makeScanPoint(2.10F, 1.0F, 12U),
    makeScanPoint(2.15F, 1.0F, 13U),
    makeScanPoint(2.20F, 1.0F, 14U)};

  const auto result =
    slam_robot_slam::estimateNormalTranslationObservability(
    points, testParameters());

  EXPECT_EQ(result.normal_count, 6U);
  EXPECT_EQ(result.rank, 1U);
}

TEST(TranslationObservability, RotatesWeakDirectionIntoTargetFrame)
{
  slam_robot_slam::TranslationObservability observability;
  observability.rank = 1U;
  observability.weak_direction = slam_robot_slam::Point2D{1.0F, 0.0F};

  const auto rotated = slam_robot_slam::rotateTranslationObservability(
    observability, 1.57079632679489661923);

  EXPECT_NEAR(rotated.weak_direction.x, 0.0, 1.0e-6);
  EXPECT_NEAR(rotated.weak_direction.y, 1.0, 1.0e-6);
}

TEST(TranslationObservability, RejectsInvalidParametersAndPoints)
{
  auto parameters = testParameters();
  parameters.beam_index_step = 0U;
  EXPECT_THROW(
    slam_robot_slam::validateTranslationObservabilityParameters(parameters),
    std::invalid_argument);

  parameters = testParameters();
  auto point = makeScanPoint(0.0F, 1.0F, 0U);
  point.range = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(
    slam_robot_slam::estimateNormalTranslationObservability(
      std::vector<slam_robot_slam::ScanPoint2D>{point}, parameters),
    std::invalid_argument);
}

}  // namespace
