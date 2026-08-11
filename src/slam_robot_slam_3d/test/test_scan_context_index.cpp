#include <cmath>
#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "slam_robot_slam_3d/scan_context_index.hpp"

namespace slam_robot_slam_3d
{
namespace
{

GlobalKeyframe makeKeyframe(
  std::size_t id, double accumulated_distance, double position_x,
  double rotation)
{
  auto scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int index = 0; index < 8; ++index) {
    const double angle = rotation + 0.35 * index;
    pcl::PointXYZI point;
    point.x = static_cast<float>((2.0 + 0.1 * index) * std::cos(angle));
    point.y = static_cast<float>((2.0 + 0.1 * index) * std::sin(angle));
    point.z = static_cast<float>(0.2 + 0.1 * (index % 3));
    scan->push_back(point);
  }
  scan->width = scan->size();
  scan->height = 1U;
  scan->is_dense = true;

  GlobalKeyframe keyframe;
  keyframe.id = id;
  keyframe.registration_scan = scan;
  keyframe.occupancy_scan = scan;
  keyframe.accumulated_distance = accumulated_distance;
  keyframe.front_end_base_pose.translation().x() = position_x;
  keyframe.base_to_sensor.translation().z() = 0.6;
  return keyframe;
}

TEST(ScanContextIndex, RetrievesDistantHistoricalPlaceAndEstimatesYaw)
{
  ScanContextParameters parameters;
  parameters.radial_bins = 10U;
  parameters.angular_bins = 36U;
  parameters.minimum_keyframe_separation = 2U;
  parameters.minimum_travel_distance = 5.0;
  ScanContextIndex index(parameters);

  EXPECT_TRUE(index.addAndQuery(makeKeyframe(0U, 0.0, 0.0, 0.0)).empty());
  EXPECT_TRUE(index.addAndQuery(makeKeyframe(1U, 5.0, 5.0, 0.0)).empty());
  const auto candidates = index.addAndQuery(
    makeKeyframe(2U, 10.0, 5.0, 0.35));

  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().keyframe_id, 0U);
  EXPECT_LT(candidates.front().descriptor_distance, 0.05);
  EXPECT_GT(std::abs(candidates.front().predicted_yaw), 0.1);
  EXPECT_EQ(index.lastQueryDiagnostics().eligible_candidates, 1U);
  EXPECT_EQ(index.lastQueryDiagnostics().accepted_candidates, 1U);
  EXPECT_EQ(index.lastQueryDiagnostics().descriptor_rejections, 0U);
}

TEST(ScanContextIndex, AcceptsScaledAndPartiallyOccludedRevisit)
{
  ScanContextParameters parameters;
  parameters.radial_bins = 10U;
  parameters.angular_bins = 36U;
  parameters.minimum_keyframe_separation = 1U;
  parameters.minimum_travel_distance = 0.0;
  parameters.maximum_descriptor_distance = 0.15;
  ScanContextIndex index(parameters);

  index.addAndQuery(makeKeyframe(0U, 0.0, 0.0, 0.0));
  auto revisit = makeKeyframe(1U, 10.0, 0.0, 0.35);
  auto scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (std::size_t point_index = 0U;
    point_index < revisit.registration_scan->size(); ++point_index)
  {
    if (point_index == 2U) {
      continue;
    }
    auto point = revisit.registration_scan->at(point_index);
    point.z *= 1.25F;
    scan->push_back(point);
  }
  scan->width = scan->size();
  scan->height = 1U;
  scan->is_dense = true;
  revisit.registration_scan = scan;

  const auto candidates = index.addAndQuery(revisit);
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().keyframe_id, 0U);
  EXPECT_LE(candidates.front().descriptor_distance, 0.15);
}

TEST(ScanContextIndex, ExcludesTemporalAndShortTravelCandidates)
{
  ScanContextParameters parameters;
  parameters.minimum_keyframe_separation = 3U;
  parameters.minimum_travel_distance = 8.0;

  ScanContextIndex temporal_index(parameters);
  temporal_index.addAndQuery(makeKeyframe(0U, 0.0, 0.0, 0.0));
  EXPECT_TRUE(
    temporal_index.addAndQuery(makeKeyframe(2U, 10.0, 5.0, 0.0)).empty());

  parameters.minimum_keyframe_separation = 1U;
  ScanContextIndex travel_index(parameters);
  travel_index.addAndQuery(makeKeyframe(0U, 0.0, 0.0, 0.0));
  EXPECT_TRUE(
    travel_index.addAndQuery(makeKeyframe(1U, 7.0, 5.0, 0.0)).empty());

  parameters.minimum_travel_distance = 0.0;
  ScanContextIndex return_index(parameters);
  return_index.addAndQuery(makeKeyframe(0U, 0.0, 0.0, 0.0));
  EXPECT_FALSE(
    return_index.addAndQuery(makeKeyframe(1U, 10.0, 0.0, 0.0)).empty());
}

TEST(ScanContextIndex, RejectsDissimilarRetrievalProposal)
{
  ScanContextParameters parameters;
  parameters.radial_bins = 10U;
  parameters.angular_bins = 36U;
  parameters.minimum_keyframe_separation = 1U;
  parameters.minimum_travel_distance = 0.0;
  parameters.maximum_descriptor_distance = 0.15;
  ScanContextIndex index(parameters);

  index.addAndQuery(makeKeyframe(0U, 0.0, 0.0, 0.0));
  auto different = makeKeyframe(1U, 10.0, 0.0, 0.0);
  auto modified_scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(
    *different.registration_scan);
  for (auto & point : *modified_scan) {
    point.x *= 1.8F;
    point.y *= 1.8F;
  }
  different.registration_scan = modified_scan;
  EXPECT_TRUE(index.addAndQuery(different).empty());
  EXPECT_EQ(index.lastQueryDiagnostics().descriptor_rejections, 1U);
  EXPECT_GT(index.lastQueryDiagnostics().best_descriptor_distance, 0.15);
}

TEST(ScanContextIndex, RejectsDescriptorThresholdOutsideNormalizedRange)
{
  ScanContextParameters parameters;
  parameters.maximum_descriptor_distance = 1.01;
  EXPECT_THROW(ScanContextIndex index(parameters), std::invalid_argument);
}

}  // namespace
}  // namespace slam_robot_slam_3d
