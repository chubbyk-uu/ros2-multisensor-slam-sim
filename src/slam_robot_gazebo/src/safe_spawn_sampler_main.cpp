// Copyright 2026 Jerry

#include "slam_robot_gazebo/safe_spawn_sampler.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

struct Arguments
{
  std::string world;
  std::string debug_pgm;
  std::size_t count{1U};
  std::uint64_t seed{0U};
  slam_robot_gazebo::SpawnSamplingParameters parameters;
};

Arguments parseArguments(int argc, char ** argv)
{
  Arguments result;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    const auto value = [&]() -> std::string {
        if (++index >= argc) {throw std::invalid_argument("missing value for " + argument);}
        return argv[index];
      };
    if (argument == "--world") {
      result.world = value();
    } else if (argument == "--count") {
      result.count = std::stoull(value());
    } else if (argument == "--seed") {
      result.seed = std::stoull(value());
    } else if (argument == "--debug-pgm") {
      result.debug_pgm = value();
    } else if (argument == "--resolution") {
      result.parameters.resolution = std::stod(value());
    } else if (argument == "--safety-margin") {
      result.parameters.safety_margin = std::stod(value());
    } else if (argument == "--robot-height") {
      result.parameters.robot_height = std::stod(value());
    } else if (argument == "--vertical-margin") {
      result.parameters.vertical_margin = std::stod(value());
    } else if (argument == "--minimum-separation") {
      result.parameters.minimum_spawn_separation = std::stod(value());
    } else if (argument == "--reference-x") {
      result.parameters.reference_x = std::stod(value());
    } else if (argument == "--reference-y") {
      result.parameters.reference_y = std::stod(value());
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  if (result.world.empty()) {throw std::invalid_argument("--world is required");}
  if (result.count == 0U) {throw std::invalid_argument("--count must be positive");}
  if (result.seed == 0U) {result.seed = slam_robot_gazebo::makeSpawnSeed();}
  return result;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto arguments = parseArguments(argc, argv);
    const auto geometry = slam_robot_gazebo::SdfWorldGeometryParser{}.parse(arguments.world);
    const slam_robot_gazebo::SafeSpawnGrid grid(geometry, arguments.parameters);
    const auto samples = grid.sample(arguments.count, arguments.seed);
    if (!arguments.debug_pgm.empty()) {grid.writeDebugPgm(arguments.debug_pgm, samples);}
    // One line, so a caller that only owns a log can still recover the record.
    std::cout << slam_robot_gazebo::formatSpawnRecord(
      arguments.world, geometry, grid, arguments.parameters, arguments.seed, samples)
              << std::endl;
    return EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "safe_spawn_sampler: " << error.what() << std::endl;
    return EXIT_FAILURE;
  }
}
