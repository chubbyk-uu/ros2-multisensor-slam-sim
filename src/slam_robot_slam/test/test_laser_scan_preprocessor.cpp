#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "slam_robot_slam/laser_scan_preprocessor.hpp"

namespace
{

constexpr double kHalfPi = 1.57079632679489661923;
constexpr double kQuarterPi = 0.78539816339744830962;

sensor_msgs::msg::LaserScan makeScan()
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = static_cast<float>(-kHalfPi);
  scan.angle_increment = static_cast<float>(kHalfPi);
  scan.range_min = 0.10F;
  scan.range_max = 12.0F;
  return scan;
}

TEST(LaserScanPreprocessor, FiltersInvalidAndOutOfRangeMeasurements)
{
  auto scan = makeScan();
  scan.ranges = {
    1.0F,
    std::numeric_limits<float>::infinity(),
    0.15F,
    2.0F,
    13.0F,
    std::numeric_limits<float>::quiet_NaN()};

  const auto points =
    slam_robot_slam::projectLaserScan(scan, 0.20, 10.0, 1U);

  ASSERT_EQ(points.size(), 2U);
  EXPECT_NEAR(points[0].x, 0.0, 1e-5);
  EXPECT_NEAR(points[0].y, -1.0, 1e-5);
  EXPECT_NEAR(points[1].x, -2.0, 1e-5);
  EXPECT_NEAR(points[1].y, 0.0, 1e-5);
}

TEST(LaserScanPreprocessor, AppliesPointStrideBeforeProjection)
{
  auto scan = makeScan();
  scan.angle_min = 0.0F;
  scan.angle_increment = static_cast<float>(kQuarterPi);
  scan.ranges = {1.0F, 2.0F, 3.0F, 4.0F, 5.0F};

  const auto points =
    slam_robot_slam::projectLaserScan(scan, 0.10, 12.0, 2U);

  ASSERT_EQ(points.size(), 3U);
  EXPECT_NEAR(points[0].x, 1.0, 1e-5);
  EXPECT_NEAR(points[0].y, 0.0, 1e-5);
  EXPECT_NEAR(points[1].x, 0.0, 1e-5);
  EXPECT_NEAR(points[1].y, 3.0, 1e-5);
  EXPECT_NEAR(points[2].x, -5.0, 1e-5);
  EXPECT_NEAR(points[2].y, 0.0, 1e-5);
}

TEST(LaserScanPreprocessor, RejectsZeroStride)
{
  const auto scan = makeScan();
  EXPECT_THROW(
    slam_robot_slam::projectLaserScan(scan, 0.10, 12.0, 0U),
    std::invalid_argument);
}

}  // namespace
