#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

#include "slam_robot_slam_3d/slam_snapshot.hpp"

namespace slam_robot_slam_3d
{
namespace
{
// A predictable name under /tmp is shared state between whatever else is
// running. Two concurrent `colcon test` invocations, or ctest parallelism
// inside this package later on, would have one case delete the file another is
// reading -- and the failure surfaces as a truncated or missing snapshot,
// which reads as a bug in the format rather than in the test setup. A unique
// directory also cleans up after a case that threw, which the manual
// std::filesystem::remove at the end of each test did not.
class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    const auto pattern =
      (std::filesystem::temp_directory_path() / "slam_snapshot_test_XXXXXX").string();
    std::vector<char> buffer(pattern.begin(), pattern.end());
    buffer.push_back('\0');
    if (::mkdtemp(buffer.data()) == nullptr) {
      throw std::runtime_error("cannot create a temporary directory for the test");
    }
    path_ = buffer.data();
  }

  ~TemporaryDirectory()
  {
    std::error_code ignored;
    std::filesystem::remove_all(path_, ignored);
  }

  TemporaryDirectory(const TemporaryDirectory &) = delete;
  TemporaryDirectory & operator=(const TemporaryDirectory &) = delete;

  std::string file(const std::string & name) const {return (path_ / name).string();}

private:
  std::filesystem::path path_;
};

GlobalKeyframe makeKeyframe(std::size_t id, double x)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  cloud->push_back(pcl::PointXYZI{1.0F, 2.0F, 0.3F, 4.0F});
  GlobalKeyframe keyframe;
  keyframe.id = id;
  keyframe.stamp = rclcpp::Time(static_cast<std::int64_t>(1000 + id), RCL_ROS_TIME);
  keyframe.registration_scan = cloud;
  auto occupancy_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  occupancy_cloud->push_back(pcl::PointXYZI{1.05F, 2.0F, 0.3F, 5.0F});
  keyframe.occupancy_scan = occupancy_cloud;
  keyframe.front_end_base_pose.translation().x() = x;
  keyframe.odom_base_pose.translation().x() = x + 0.1;
  keyframe.base_to_sensor.translation().z() = 0.2;
  keyframe.pose_covariance(0, 0) = 0.123;
  keyframe.accumulated_distance = x;
  keyframe.match_accepted = true;
  keyframe.correspondence_count = 42U;
  keyframe.rmse = 0.03;
  return keyframe;
}
}

TEST(SlamSnapshot, RoundTripsAllPersistentState)
{
  const TemporaryDirectory directory;
  const auto path = directory.file("snapshot.bin");
  SlamSnapshot source;
  source.keyframes = {makeKeyframe(0U, 0.0), makeKeyframe(1U, 1.0)};
  source.loop_constraints.push_back({0U, 1U, Eigen::Isometry3d::Identity()});
  source.optimized_base_poses = {
    Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity()};
  source.optimized_base_poses.back().translation().x() = 0.9;
  source.occupancy_projection.input_voxel_leaf_size = 0.04;
  source.occupancy_projection.maximum_ray_range = 7.5;

  saveSlamSnapshot(path, source);
  const auto restored = loadSlamSnapshot(path);

  ASSERT_EQ(restored.keyframes.size(), 2U);
  ASSERT_EQ(restored.loop_constraints.size(), 1U);
  ASSERT_EQ(restored.optimized_base_poses.size(), 2U);
  EXPECT_DOUBLE_EQ(restored.keyframes[1].front_end_base_pose.translation().x(), 1.0);
  EXPECT_DOUBLE_EQ(restored.keyframes[1].pose_covariance(0, 0), 0.123);
  EXPECT_EQ(restored.keyframes[1].correspondence_count, 42U);
  EXPECT_FLOAT_EQ(restored.keyframes[0].registration_scan->front().intensity, 4.0F);
  EXPECT_FLOAT_EQ(restored.keyframes[0].occupancy_scan->front().intensity, 5.0F);
  EXPECT_DOUBLE_EQ(restored.optimized_base_poses[1].translation().x(), 0.9);
  EXPECT_DOUBLE_EQ(restored.occupancy_projection.input_voxel_leaf_size, 0.04);
  EXPECT_DOUBLE_EQ(restored.occupancy_projection.maximum_ray_range, 7.5);
}

TEST(SlamSnapshot, RejectsEmptyState)
{
  EXPECT_THROW(saveSlamSnapshot("unused", {}), std::invalid_argument);
}

TEST(SlamSnapshot, RoundTripsTheFrontEndFrameCorrection)
{
  // Without this the frame the keyframes were written in is lost, and mapping
  // resumed from the snapshot mixes those poses with new ones expressed in the
  // map frame.
  const TemporaryDirectory directory;
  const auto path = directory.file("map_from_local.bin");
  SlamSnapshot snapshot;
  snapshot.keyframes.push_back(makeKeyframe(0U, 0.0));
  snapshot.optimized_base_poses.push_back(Eigen::Isometry3d::Identity());
  snapshot.map_from_local.translation() = Eigen::Vector3d(1.5, -2.5, 0.0);

  saveSlamSnapshot(path, snapshot);
  const auto restored = loadSlamSnapshot(path);

  EXPECT_NEAR(restored.map_from_local.translation().x(), 1.5, 1.0e-9);
  EXPECT_NEAR(restored.map_from_local.translation().y(), -2.5, 1.0e-9);
}


void expectLegacyVersionRejected(std::uint32_t legacy_version)
{
  const TemporaryDirectory directory;
  const auto path = directory.file("v" + std::to_string(legacy_version) + ".bin");
  SlamSnapshot source;
  source.keyframes = {makeKeyframe(0U, 0.0)};
  source.optimized_base_poses = {Eigen::Isometry3d::Identity()};
  saveSlamSnapshot(path, source);

  std::fstream file(path, std::ios::binary | std::ios::in | std::ios::out);
  file.seekp(sizeof(std::uint64_t));
  file.write(
    reinterpret_cast<const char *>(&legacy_version), sizeof(legacy_version));
  file.close();

  // Not just "it threw". Editing the version also invalidates the checksum, so
  // an implementation that verified the checksum first would still throw here
  // while reporting corruption for a file that is merely old -- the same
  // misdirection version 3 produced, one layer down.
  try {
    loadSlamSnapshot(path);
    ADD_FAILURE() << "a version " << legacy_version << " snapshot was accepted";
  } catch (const std::runtime_error & error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("unsupported SLAM snapshot version"), std::string::npos)
      << "reported as: " << message;
    EXPECT_EQ(message.find("checksum"), std::string::npos)
      << "reported as: " << message;
  }
}

TEST(SlamSnapshot, RejectsVersionOneSnapshots)
{
  expectLegacyVersionRejected(1U);
}

TEST(SlamSnapshot, RejectsVersionTwoSnapshots)
{
  expectLegacyVersionRejected(2U);
}

TEST(SlamSnapshot, RejectsVersionThreeSnapshots)
{
  // Version 3 shipped in two incompatible layouts: the second inserted the
  // occupancy projection contract ahead of the keyframe count. Reading an
  // early one under the later layout takes the keyframe count for a contract
  // and blames the contract for a format change, so the version was bumped
  // rather than reused.
  expectLegacyVersionRejected(3U);
}

TEST(SlamSnapshot, RejectsTruncatedSnapshots)
{
  const TemporaryDirectory directory;
  const auto path = directory.file("truncated.bin");
  SlamSnapshot source;
  source.keyframes = {makeKeyframe(0U, 0.0)};
  source.optimized_base_poses = {Eigen::Isometry3d::Identity()};
  saveSlamSnapshot(path, source);
  std::filesystem::resize_file(path, std::filesystem::file_size(path) / 2U);

  EXPECT_THROW(loadSlamSnapshot(path), std::runtime_error);
}

TEST(SlamSnapshot, RejectsACorruptedPayload)
{
  // The failure a length check cannot see: the file is exactly as long as it
  // should be, and one byte inside it is wrong. Restoring from it would put a
  // silently wrong prior map under a robot that then localizes against it.
  const TemporaryDirectory directory;
  const auto path = directory.file("corrupt.bin");
  SlamSnapshot source;
  source.keyframes = {makeKeyframe(0U, 0.0)};
  source.optimized_base_poses = {Eigen::Isometry3d::Identity()};
  saveSlamSnapshot(path, source);
  const auto size = std::filesystem::file_size(path);

  std::fstream file(path, std::ios::binary | std::ios::in | std::ios::out);
  const auto middle = static_cast<std::streamoff>(size / 2U);
  file.seekg(middle);
  char byte = 0;
  file.read(&byte, 1);
  byte = static_cast<char>(byte ^ 0x01);
  file.seekp(middle);
  file.write(&byte, 1);
  file.close();

  EXPECT_EQ(std::filesystem::file_size(path), size);
  EXPECT_THROW(loadSlamSnapshot(path), std::runtime_error);
}

TEST(SlamSnapshot, TheTrailerIsTheChecksumOfEverythingBeforeIt)
{
  // Recomputed here rather than asserted through a round trip, because a round
  // trip passes for any writer and reader that agree -- a constant trailer, or
  // one covering only the header, would satisfy it. This pins what the bytes
  // on disk have to be, so the reader's check has something real to verify.
  const TemporaryDirectory directory;
  const auto path = directory.file("checksummed.bin");
  SlamSnapshot source;
  source.keyframes = {makeKeyframe(0U, 0.0)};
  source.optimized_base_poses = {Eigen::Isometry3d::Identity()};
  saveSlamSnapshot(path, source);

  std::ifstream file(path, std::ios::binary);
  const std::string contents(
    (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  ASSERT_GT(contents.size(), sizeof(std::uint64_t));
  const std::size_t payload_size = contents.size() - sizeof(std::uint64_t);

  std::uint64_t expected = 0xCBF29CE484222325ULL;
  for (std::size_t index = 0U; index < payload_size; ++index) {
    expected ^= static_cast<unsigned char>(contents[index]);
    expected *= 0x100000001B3ULL;
  }
  std::uint64_t stored = 0U;
  std::memcpy(&stored, contents.data() + payload_size, sizeof(stored));

  EXPECT_EQ(stored, expected);
}

TEST(SlamSnapshot, ReportsAVersionFourFileAsAVersionRatherThanAsCorruption)
{
  // Version 4 files exist and have no checksum trailer, so their last eight
  // bytes are payload. Verifying the checksum before the version would read
  // those as a trailer, fail, and send the reader looking for a storage fault.
  const TemporaryDirectory directory;
  const auto path = directory.file("version_four.bin");
  const std::uint64_t magic = 0x534C414D33445331ULL;
  const std::uint32_t version = 4U;
  std::ofstream file(path, std::ios::binary);
  file.write(reinterpret_cast<const char *>(&magic), sizeof(magic));
  file.write(reinterpret_cast<const char *>(&version), sizeof(version));
  const std::string payload(64U, '\x7f');
  file.write(payload.data(), static_cast<std::streamsize>(payload.size()));
  file.close();

  try {
    loadSlamSnapshot(path);
    ADD_FAILURE() << "a version 4 snapshot was accepted";
  } catch (const std::runtime_error & error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("unsupported SLAM snapshot version 4"), std::string::npos)
      << "reported as: " << message;
    EXPECT_EQ(message.find("checksum"), std::string::npos)
      << "reported as: " << message;
  }
}

}  // namespace slam_robot_slam_3d
