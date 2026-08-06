#include <cmath>
#include <memory>

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
  keyframe.filtered_scan = scan;
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

}  // namespace
}  // namespace slam_robot_slam_3d
