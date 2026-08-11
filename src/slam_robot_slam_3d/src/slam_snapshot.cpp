#include "slam_robot_slam_3d/slam_snapshot.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>

#include <pcl/common/point_tests.h>

namespace slam_robot_slam_3d
{
namespace
{
constexpr std::uint64_t kMagic = 0x534C414D33445331ULL;
// Bumped whenever the byte layout changes, not only when a field is added at
// the end. Version 3 shipped twice with different layouts -- the occupancy
// projection contract was later inserted between the version field and the
// keyframe count -- and the older files then failed while reporting an invalid
// contract, which points at the wrong thing entirely. A version field exists
// precisely so a layout change announces itself.
constexpr std::uint32_t kVersion = 4U;
constexpr std::uint64_t kMaximumKeyframes = 1000000U;
constexpr std::uint64_t kMaximumPointsPerKeyframe = 10000000U;
constexpr std::uint64_t kMaximumConstraints = 10000000U;

template<typename T>
void writeValue(std::ostream & output, const T & value)
{
  output.write(reinterpret_cast<const char *>(&value), sizeof(T));
}

template<typename T>
void readValue(std::istream & input, T & value)
{
  if (!input.read(reinterpret_cast<char *>(&value), sizeof(T))) {
    throw std::runtime_error("truncated SLAM snapshot");
  }
}

void writePose(std::ostream & output, const Eigen::Isometry3d & pose)
{
  if (!pose.matrix().allFinite()) {
    throw std::invalid_argument("snapshot pose must be finite");
  }
  for (Eigen::Index row = 0; row < 4; ++row) {
    for (Eigen::Index column = 0; column < 4; ++column) {
      writeValue(output, pose.matrix()(row, column));
    }
  }
}

Eigen::Isometry3d readPose(std::istream & input)
{
  Eigen::Matrix4d matrix;
  for (Eigen::Index row = 0; row < 4; ++row) {
    for (Eigen::Index column = 0; column < 4; ++column) {
      readValue(input, matrix(row, column));
    }
  }
  if (!matrix.allFinite()) {
    throw std::runtime_error("snapshot pose is not finite");
  }
  Eigen::Isometry3d pose;
  pose.matrix() = matrix;
  return pose;
}

void writeOccupancyProjectionContract(
  std::ostream & output, const OccupancyProjectionContract & contract)
{
  validateOccupancyProjectionContract(contract);
  writeValue(output, contract.input_voxel_leaf_size);
  writeValue(output, contract.minimum_obstacle_height);
  writeValue(output, contract.maximum_obstacle_height);
  writeValue(output, contract.maximum_ray_range);
  writeValue(output, static_cast<std::uint8_t>(contract.force_planar_motion ? 1U : 0U));
}

OccupancyProjectionContract readOccupancyProjectionContract(std::istream & input)
{
  OccupancyProjectionContract contract;
  std::uint8_t force_planar_motion{0U};
  readValue(input, contract.input_voxel_leaf_size);
  readValue(input, contract.minimum_obstacle_height);
  readValue(input, contract.maximum_obstacle_height);
  readValue(input, contract.maximum_ray_range);
  readValue(input, force_planar_motion);
  if (force_planar_motion > 1U) {
    throw std::runtime_error("invalid snapshot planar-motion contract");
  }
  contract.force_planar_motion = force_planar_motion != 0U;
  try {
    validateOccupancyProjectionContract(contract);
  } catch (const std::invalid_argument &) {
    throw std::runtime_error("invalid snapshot occupancy projection contract");
  }
  return contract;
}

void writePointCloud(
  std::ostream & output,
  const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> & cloud)
{
  if (!cloud || cloud->empty() || cloud->size() > kMaximumPointsPerKeyframe) {
    throw std::invalid_argument("snapshot keyframe cloud must not be empty");
  }
  writeValue(output, static_cast<std::uint64_t>(cloud->size()));
  for (const auto & point : *cloud) {
    if (!pcl::isFinite(point)) {
      throw std::invalid_argument("snapshot keyframe cloud must be finite");
    }
    writeValue(output, point.x);
    writeValue(output, point.y);
    writeValue(output, point.z);
    writeValue(output, point.intensity);
  }
}

std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> readPointCloud(
  std::istream & input)
{
  std::uint64_t point_count{0U};
  readValue(input, point_count);
  if (point_count == 0U || point_count > kMaximumPointsPerKeyframe) {
    throw std::runtime_error("invalid snapshot keyframe point count");
  }
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  cloud->resize(static_cast<std::size_t>(point_count));
  for (auto & point : *cloud) {
    readValue(input, point.x);
    readValue(input, point.y);
    readValue(input, point.z);
    readValue(input, point.intensity);
    if (!pcl::isFinite(point)) {
      throw std::runtime_error("snapshot keyframe cloud is not finite");
    }
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1U;
  cloud->is_dense = true;
  return cloud;
}

void writeKeyframe(std::ostream & output, const GlobalKeyframe & keyframe)
{
  const auto stamp = keyframe.stamp.nanoseconds();
  const std::uint8_t flags =
    (keyframe.match_accepted ? 1U : 0U) |
    (keyframe.translation_degenerate ? 2U : 0U) |
    (keyframe.planar_degenerate ? 4U : 0U) |
    (keyframe.yaw_degenerate ? 8U : 0U);
  writeValue(output, stamp);
  writePose(output, keyframe.front_end_base_pose);
  writePose(output, keyframe.odom_base_pose);
  writePose(output, keyframe.base_to_sensor);
  for (Eigen::Index row = 0; row < 6; ++row) {
    for (Eigen::Index column = 0; column < 6; ++column) {
      writeValue(output, keyframe.pose_covariance(row, column));
    }
  }
  writeValue(output, keyframe.accumulated_distance);
  writeValue(output, flags);
  writeValue(output, static_cast<std::uint64_t>(keyframe.correspondence_count));
  writeValue(output, keyframe.rmse);
  writePointCloud(output, keyframe.registration_scan);
  writePointCloud(output, keyframe.occupancy_scan);
}

GlobalKeyframe readKeyframe(std::istream & input, std::size_t id)
{
  GlobalKeyframe keyframe;
  std::int64_t stamp{0};
  std::uint8_t flags{0U};
  std::uint64_t correspondences{0U};
  readValue(input, stamp);
  keyframe.front_end_base_pose = readPose(input);
  keyframe.odom_base_pose = readPose(input);
  keyframe.base_to_sensor = readPose(input);
  for (Eigen::Index row = 0; row < 6; ++row) {
    for (Eigen::Index column = 0; column < 6; ++column) {
      readValue(input, keyframe.pose_covariance(row, column));
    }
  }
  readValue(input, keyframe.accumulated_distance);
  readValue(input, flags);
  readValue(input, correspondences);
  readValue(input, keyframe.rmse);
  keyframe.id = id;
  keyframe.stamp = rclcpp::Time(stamp, RCL_ROS_TIME);
  keyframe.registration_scan = readPointCloud(input);
  keyframe.occupancy_scan = readPointCloud(input);
  keyframe.match_accepted = (flags & 1U) != 0U;
  keyframe.translation_degenerate = (flags & 2U) != 0U;
  keyframe.planar_degenerate = (flags & 4U) != 0U;
  keyframe.yaw_degenerate = (flags & 8U) != 0U;
  keyframe.correspondence_count = static_cast<std::size_t>(correspondences);
  return keyframe;
}
}  // namespace

void saveSlamSnapshot(const std::string & path, const SlamSnapshot & snapshot)
{
  if (path.empty() || snapshot.keyframes.empty() ||
    snapshot.optimized_base_poses.size() != snapshot.keyframes.size())
  {
    throw std::invalid_argument("snapshot path, keyframes and poses are inconsistent");
  }
  validateOccupancyProjectionContract(snapshot.occupancy_projection);
  const std::filesystem::path destination(path);
  if (!destination.parent_path().empty()) {
    std::filesystem::create_directories(destination.parent_path());
  }
  const auto temporary = destination.string() + ".tmp";
  std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot create temporary SLAM snapshot");
  }
  writeValue(output, kMagic);
  writeValue(output, kVersion);
  writeOccupancyProjectionContract(output, snapshot.occupancy_projection);
  writeValue(output, static_cast<std::uint64_t>(snapshot.keyframes.size()));
  for (const auto & keyframe : snapshot.keyframes) {
    writeKeyframe(output, keyframe);
  }
  writeValue(output, static_cast<std::uint64_t>(snapshot.loop_constraints.size()));
  for (const auto & constraint : snapshot.loop_constraints) {
    writeValue(output, static_cast<std::uint64_t>(constraint.source_id));
    writeValue(output, static_cast<std::uint64_t>(constraint.target_id));
    writePose(output, constraint.relative_pose);
  }
  for (const auto & pose : snapshot.optimized_base_poses) {
    writePose(output, pose);
  }
  writePose(output, snapshot.map_from_local);
  output.flush();
  if (!output) {
    throw std::runtime_error("failed to write SLAM snapshot");
  }
  output.close();
  std::filesystem::rename(temporary, destination);
}

SlamSnapshot loadSlamSnapshot(const std::string & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("cannot open SLAM snapshot");
  }
  std::uint64_t magic{0U};
  std::uint32_t version{0U};
  std::uint64_t keyframe_count{0U};
  readValue(input, magic);
  readValue(input, version);
  if (magic != kMagic) {
    throw std::runtime_error("invalid SLAM snapshot magic");
  }
  if (version != kVersion) {
    throw std::runtime_error(
            "unsupported SLAM snapshot version " + std::to_string(version) +
            "; remap to generate a version " + std::to_string(kVersion) +
            " snapshot");
  }
  SlamSnapshot result;
  result.occupancy_projection = readOccupancyProjectionContract(input);
  readValue(input, keyframe_count);
  if (keyframe_count == 0U || keyframe_count > kMaximumKeyframes) {
    throw std::runtime_error("invalid SLAM snapshot keyframe count");
  }
  result.keyframes.reserve(static_cast<std::size_t>(keyframe_count));
  for (std::uint64_t id = 0U; id < keyframe_count; ++id) {
    result.keyframes.push_back(readKeyframe(input, static_cast<std::size_t>(id)));
  }
  std::uint64_t constraint_count{0U};
  readValue(input, constraint_count);
  if (constraint_count > kMaximumConstraints) {
    throw std::runtime_error("invalid snapshot constraint count");
  }
  result.loop_constraints.reserve(static_cast<std::size_t>(constraint_count));
  for (std::uint64_t index = 0U; index < constraint_count; ++index) {
    std::uint64_t source{0U};
    std::uint64_t target{0U};
    readValue(input, source);
    readValue(input, target);
    if (source >= keyframe_count || target >= keyframe_count || source == target) {
      throw std::runtime_error("invalid snapshot loop constraint");
    }
    result.loop_constraints.push_back({
        static_cast<std::size_t>(source), static_cast<std::size_t>(target),
        readPose(input)});
  }
  result.optimized_base_poses.reserve(static_cast<std::size_t>(keyframe_count));
  for (std::uint64_t index = 0U; index < keyframe_count; ++index) {
    result.optimized_base_poses.push_back(readPose(input));
  }
  result.map_from_local = readPose(input);
  return result;
}
}  // namespace slam_robot_slam_3d
