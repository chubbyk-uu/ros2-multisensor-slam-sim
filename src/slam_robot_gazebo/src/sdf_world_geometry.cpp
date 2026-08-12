// Copyright 2026 Jerry

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "slam_robot_gazebo/safe_spawn_sampler.hpp"

#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Collision.hh>
#include <sdf/Cone.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Geometry.hh>
#include <sdf/Link.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>
#include <sdf/Plane.hh>
#include <sdf/Root.hh>
#include <sdf/Sphere.hh>
#include <sdf/World.hh>

namespace slam_robot_gazebo
{
namespace
{

constexpr double kHorizontalTolerance = 1.0e-6;

std::string errorsToString(const sdf::Errors & errors)
{
  std::ostringstream stream;
  for (std::size_t index = 0U; index < errors.size(); ++index) {
    if (index != 0U) {stream << "; ";}
    stream << errors[index].Message();
  }
  return stream.str();
}

template<typename SemanticPoseOwner>
gz::math::Pose3d resolveLocalPose(const SemanticPoseOwner & owner, const std::string & name)
{
  gz::math::Pose3d pose;
  const auto errors = owner.SemanticPose().Resolve(pose);
  if (!errors.empty()) {
    throw std::runtime_error(
            "cannot resolve local pose of " + name + ": " + errorsToString(errors));
  }
  return pose;
}

std::array<double, 6> transformedBounds(
  const gz::math::Pose3d & pose, const gz::math::Vector3d & minimum,
  const gz::math::Vector3d & maximum)
{
  std::array<double, 6> bounds{
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity()};
  for (int x = 0; x < 2; ++x) {
    for (int y = 0; y < 2; ++y) {
      for (int z = 0; z < 2; ++z) {
        const gz::math::Vector3d local{
          x == 0 ? minimum.X() : maximum.X(),
          y == 0 ? minimum.Y() : maximum.Y(),
          z == 0 ? minimum.Z() : maximum.Z()};
        const auto world = pose.Pos() + pose.Rot().RotateVector(local);
        bounds[0] = std::min(bounds[0], world.X());
        bounds[1] = std::min(bounds[1], world.Y());
        bounds[2] = std::min(bounds[2], world.Z());
        bounds[3] = std::max(bounds[3], world.X());
        bounds[4] = std::max(bounds[4], world.Y());
        bounds[5] = std::max(bounds[5], world.Z());
      }
    }
  }
  return bounds;
}

CollisionFootprint rectangleFromBounds(
  const std::string & name, const std::array<double, 6> & bounds)
{
  CollisionFootprint result;
  result.type = FootprintType::kRectangle;
  result.name = name;
  result.center = {(bounds[0] + bounds[3]) * 0.5, (bounds[1] + bounds[4]) * 0.5};
  result.size_x = bounds[3] - bounds[0];
  result.size_y = bounds[4] - bounds[1];
  result.minimum_z = bounds[2];
  result.maximum_z = bounds[5];
  return result;
}

CollisionFootprint boxFootprint(
  const sdf::Collision & collision, const gz::math::Pose3d & pose,
  const gz::math::Vector3d & size)
{
  const auto bounds = transformedBounds(pose, -size * 0.5, size * 0.5);
  if (std::abs(pose.Rot().Roll()) > kHorizontalTolerance ||
    std::abs(pose.Rot().Pitch()) > kHorizontalTolerance)
  {
    return rectangleFromBounds(collision.Name(), bounds);
  }
  CollisionFootprint result;
  result.type = FootprintType::kRectangle;
  result.name = collision.Name();
  result.center = {pose.Pos().X(), pose.Pos().Y()};
  result.yaw = pose.Rot().Yaw();
  result.size_x = size.X();
  result.size_y = size.Y();
  result.minimum_z = bounds[2];
  result.maximum_z = bounds[5];
  return result;
}

CollisionFootprint circleFootprint(
  const sdf::Collision & collision, const gz::math::Pose3d & pose,
  double radius, double half_height)
{
  // A tilted axial shape gets a conservative sphere-like XY projection. The
  // supported project worlds keep cylinders vertical; this branch is what
  // prevents a future tilted collision from being silently underestimated.
  const bool tilted = std::abs(pose.Rot().Roll()) > kHorizontalTolerance ||
    std::abs(pose.Rot().Pitch()) > kHorizontalTolerance;
  CollisionFootprint result;
  result.type = FootprintType::kCircle;
  result.name = collision.Name();
  result.center = {pose.Pos().X(), pose.Pos().Y()};
  result.radius = tilted ? std::hypot(radius, half_height) : radius;
  result.minimum_z = pose.Pos().Z() - (tilted ? result.radius : half_height);
  result.maximum_z = pose.Pos().Z() + (tilted ? result.radius : half_height);
  return result;
}

void extendBounds(Bounds2D & bounds, const SupportSurface & support, bool & initialized);

// Reported, not decided. Whether a thin flat horizontal body is the floor
// depends on where the robot starts, which the parser does not know; the grid
// resolves it against the reference point. The old absolute -0.10..0.10 m band
// tried to decide it here and could only be right for worlds authored at z=0 --
// and it treated planes and boxes inconsistently, since the plane branch never
// had a band at all.
void addSupportCandidate(
  ParsedWorldGeometry & result, const SupportSurface & surface,
  const CollisionFootprint & body, bool & bounds_initialized)
{
  result.support_candidates.push_back({surface, body});
  extendBounds(result.sampling_bounds, surface, bounds_initialized);
}

void extendBounds(Bounds2D & bounds, const SupportSurface & support, bool & initialized)
{
  const double cosine = std::abs(std::cos(support.yaw));
  const double sine = std::abs(std::sin(support.yaw));
  const double half_x = (cosine * support.size_x + sine * support.size_y) * 0.5;
  const double half_y = (sine * support.size_x + cosine * support.size_y) * 0.5;
  const Bounds2D candidate{
    support.center.x - half_x, support.center.y - half_y,
    support.center.x + half_x, support.center.y + half_y};
  if (!initialized) {
    bounds = candidate;
    initialized = true;
    return;
  }
  bounds.minimum_x = std::min(bounds.minimum_x, candidate.minimum_x);
  bounds.minimum_y = std::min(bounds.minimum_y, candidate.minimum_y);
  bounds.maximum_x = std::max(bounds.maximum_x, candidate.maximum_x);
  bounds.maximum_y = std::max(bounds.maximum_y, candidate.maximum_y);
}

// Thin, flat and horizontal is all the parser can say. The height at which it
// sits is recorded, not judged.
bool addSupportCandidateIfFlat(
  ParsedWorldGeometry & result, const CollisionFootprint & footprint,
  const gz::math::Pose3d & pose, bool & bounds_initialized)
{
  const bool horizontal = std::abs(pose.Rot().Roll()) <= kHorizontalTolerance &&
    std::abs(pose.Rot().Pitch()) <= kHorizontalTolerance;
  const bool thin = footprint.maximum_z - footprint.minimum_z <= 0.20;
  if (!horizontal || !thin || footprint.type != FootprintType::kRectangle) {
    return false;
  }
  SupportSurface support;
  support.center = footprint.center;
  support.yaw = footprint.yaw;
  support.size_x = footprint.size_x;
  support.size_y = footprint.size_y;
  support.height = footprint.maximum_z;
  addSupportCandidate(result, support, footprint, bounds_initialized);
  return true;
}

void addCollision(
  ParsedWorldGeometry & result, const sdf::Collision & collision,
  const gz::math::Pose3d & link_world_pose, bool & bounds_initialized,
  bool dynamic)
{
  const auto pose = link_world_pose * resolveLocalPose(collision, collision.Name());
  const auto * geometry = collision.Geom();
  if (geometry == nullptr) {
    throw std::runtime_error("collision has no geometry: " + collision.Name());
  }
  if (dynamic) {++result.non_static_collisions;}
  if (geometry->Type() == sdf::GeometryType::PLANE) {
    const auto * plane = geometry->PlaneShape();
    if (plane == nullptr || std::abs(plane->Normal().Z()) < 0.99 ||
      plane->Size().X() <= 0.0 || plane->Size().Y() <= 0.0)
    {
      throw std::runtime_error("only finite horizontal support planes are supported");
    }
    SupportSurface support;
    support.center = {pose.Pos().X(), pose.Pos().Y()};
    support.yaw = pose.Rot().Yaw();
    support.size_x = plane->Size().X();
    support.size_y = plane->Size().Y();
    support.height = pose.Pos().Z();
    // A plane has no volume, so if it turns out not to be the floor it stands
    // in as a zero-thickness rectangle at its own height -- which is what a
    // raised platform's deck is, seen from below.
    CollisionFootprint body;
    body.type = FootprintType::kRectangle;
    body.name = collision.Name();
    body.center = support.center;
    body.yaw = support.yaw;
    body.size_x = support.size_x;
    body.size_y = support.size_y;
    body.minimum_z = support.height;
    body.maximum_z = support.height;
    body.dynamic = dynamic;
    addSupportCandidate(result, support, body, bounds_initialized);
    return;
  }

  CollisionFootprint footprint;
  switch (geometry->Type()) {
    case sdf::GeometryType::BOX:
      footprint = boxFootprint(collision, pose, geometry->BoxShape()->Size());
      break;
    case sdf::GeometryType::CYLINDER:
      footprint = circleFootprint(
        collision, pose, geometry->CylinderShape()->Radius(),
        geometry->CylinderShape()->Length() * 0.5);
      break;
    case sdf::GeometryType::SPHERE:
      footprint = circleFootprint(
        collision, pose, geometry->SphereShape()->Radius(),
        geometry->SphereShape()->Radius());
      break;
    case sdf::GeometryType::CAPSULE:
      footprint = circleFootprint(
        collision, pose, geometry->CapsuleShape()->Radius(),
        geometry->CapsuleShape()->Length() * 0.5 +
        geometry->CapsuleShape()->Radius());
      break;
    case sdf::GeometryType::CONE:
      footprint = circleFootprint(
        collision, pose, geometry->ConeShape()->Radius(),
        geometry->ConeShape()->Length() * 0.5);
      break;
    case sdf::GeometryType::MESH:
      {
        const auto * shape = geometry->MeshShape();
        const std::string filename = shape->FilePath().empty() ? shape->Uri() : shape->FilePath();
        const auto * mesh = gz::common::MeshManager::Instance()->Load(filename);
        if (mesh == nullptr) {
          throw std::runtime_error("cannot load collision mesh: " + filename);
        }
        const auto scale = shape->Scale();
        const auto minimum = mesh->Min() * scale;
        const auto maximum = mesh->Max() * scale;
        footprint = rectangleFromBounds(
        collision.Name(), transformedBounds(pose, minimum, maximum));
        break;
      }
    default:
      throw std::runtime_error(
              "unsupported collision geometry in safe-spawn sampler: " + collision.Name());
  }
  footprint.dynamic = dynamic;
  // A movable body is never treated as floor, however flat it lies: a pallet
  // that happens to be thin and low is not ground, and the moment it slides
  // the "support" underneath a spawn is gone.
  if (dynamic || !addSupportCandidateIfFlat(result, footprint, pose, bounds_initialized)) {
    result.obstacles.push_back(std::move(footprint));
  }
}

void addModel(
  ParsedWorldGeometry & result, const sdf::Model & model,
  const gz::math::Pose3d & parent_world_pose, bool & bounds_initialized,
  const WorldParsingPolicy & policy, bool inherited_dynamic)
{
  // A model Gazebo may move used to be skipped outright, which meant a crate
  // that is simply not marked static was invisible and could be spawned into.
  // Its SDF pose is exact at t=0, which is the only instant that matters for
  // choosing a spawn, so avoiding it conservatively is both safer than
  // ignoring it and more usable than refusing the world.
  const bool dynamic = inherited_dynamic || !model.Static();
  if (dynamic && policy.reject_non_static) {
    throw std::runtime_error(
            "world contains a non-static model and reject_non_static is set: " +
            model.Name());
  }
  const auto model_world_pose =
    parent_world_pose * resolveLocalPose(model, model.Name());
  for (std::uint64_t link_index = 0U; link_index < model.LinkCount(); ++link_index) {
    const auto * link = model.LinkByIndex(link_index);
    const auto link_world_pose =
      model_world_pose * resolveLocalPose(*link, link->Name());
    for (std::uint64_t collision_index = 0U;
      collision_index < link->CollisionCount(); ++collision_index)
    {
      addCollision(
        result, *link->CollisionByIndex(collision_index), link_world_pose,
        bounds_initialized, dynamic);
    }
  }
  for (std::uint64_t model_index = 0U; model_index < model.ModelCount(); ++model_index) {
    addModel(
      result, *model.ModelByIndex(model_index), model_world_pose,
      bounds_initialized, policy, dynamic);
  }
}

}  // namespace

ParsedWorldGeometry SdfWorldGeometryParser::parse(const std::string & world_path) const
{
  if (world_path.empty() || !std::filesystem::exists(world_path)) {
    throw std::invalid_argument("SDF world does not exist: " + world_path);
  }
  sdf::Root root;
  const auto errors = root.Load(world_path);
  if (!errors.empty()) {
    throw std::runtime_error("failed to parse SDF world: " + errorsToString(errors));
  }
  if (root.WorldCount() != 1U) {
    throw std::runtime_error("safe-spawn sampler requires exactly one world per SDF file");
  }
  const auto * world = root.WorldByIndex(0U);
  ParsedWorldGeometry result;
  result.world_name = world->Name();
  bool bounds_initialized = false;
  for (std::uint64_t index = 0U; index < world->ModelCount(); ++index) {
    addModel(
      result, *world->ModelByIndex(index), gz::math::Pose3d::Zero,
      bounds_initialized, policy_, false);
  }
  if (result.support_candidates.empty() || !bounds_initialized) {
    throw std::runtime_error("world contains no thin horizontal collision that could be a floor");
  }
  return result;
}

}  // namespace slam_robot_gazebo
