// Copyright 2026 Jerry

#include <string>
#include <vector>

#include "slam_robot_gazebo/safe_spawn_sampler.hpp"

#include <nlohmann/json.hpp>

namespace slam_robot_gazebo
{
namespace
{

// Every field here moves a pose. The robot envelope and the spawn lift have no
// command-line flag on purpose -- they are a contract with the robot model, not
// a knob -- but they still have to be written down, because a record that omits
// them cannot tell a rerun from a different sampler.
nlohmann::json parametersToJson(const SpawnSamplingParameters & parameters)
{
  return {
    {"resolution", parameters.resolution},
    {"robot_circumscribed_radius", parameters.robot_circumscribed_radius},
    {"safety_margin", parameters.safety_margin},
    {"robot_height", parameters.robot_height},
    {"vertical_margin", parameters.vertical_margin},
    {"minimum_spawn_separation", parameters.minimum_spawn_separation},
    {"spawn_z", parameters.spawn_z},
    {"reference_x", parameters.reference_x},
    {"reference_y", parameters.reference_y},
    {"maximum_grid_cells", parameters.maximum_grid_cells},
  };
}

nlohmann::json boundsToJson(const Bounds2D & bounds)
{
  return {
    {"minimum_x", bounds.minimum_x},
    {"minimum_y", bounds.minimum_y},
    {"maximum_x", bounds.maximum_x},
    {"maximum_y", bounds.maximum_y},
  };
}

}  // namespace

std::string formatSpawnRecord(
  const std::string & world_path,
  const ParsedWorldGeometry & geometry,
  const SafeSpawnGrid & grid,
  const SpawnSamplingParameters & parameters,
  std::uint64_t seed,
  const std::vector<SpawnPose> & poses)
{
  nlohmann::json encoded_poses = nlohmann::json::array();
  for (const auto & pose : poses) {
    encoded_poses.push_back(
      {{"x", pose.x}, {"y", pose.y}, {"z", pose.z}, {"yaw", pose.yaw}});
  }
  const nlohmann::json record{
    {"schema_version", kSpawnRecordSchemaVersion},
    {"world", world_path},
    {"world_name", geometry.world_name},
    {"seed", seed},
    {"parameters", parametersToJson(parameters)},
    {"sampling_bounds", boundsToJson(grid.bounds())},
    {"safe_cells", grid.safeCellCount()},
    {"poses", encoded_poses},
  };
  // Full double precision rather than a fixed six places: a truncated pose is
  // not the pose the simulator was given.
  return record.dump();
}

}  // namespace slam_robot_gazebo
