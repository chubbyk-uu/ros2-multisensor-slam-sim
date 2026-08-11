#include <memory>

#include <gtest/gtest.h>

#include "slam_robot_slam_3d/loop_closure_verifier.hpp"

namespace slam_robot_slam_3d
{
namespace
{

pcl::PointCloud<pcl::PointXYZI> makeStructuredCloud()
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (double first = -1.5; first <= 1.5; first += 0.15) {
    for (double second = -1.0; second <= 1.0; second += 0.15) {
      cloud.push_back({
            static_cast<float>(first), static_cast<float>(second), 0.0F, 1.0F});
      cloud.push_back({
            2.0F, static_cast<float>(first),
            static_cast<float>(second + 1.0), 2.0F});
      cloud.push_back({
            static_cast<float>(first), -1.5F,
            static_cast<float>(second + 1.0), 3.0F});
    }
  }
  cloud.width = cloud.size();
  cloud.height = 1U;
  cloud.is_dense = true;
  return cloud;
}

pcl::PointCloud<pcl::PointXYZI> makeGroundCloud()
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (double x = -2.0; x <= 2.0; x += 0.10) {
    for (double y = -2.0; y <= 2.0; y += 0.10) {
      cloud.push_back({static_cast<float>(x), static_cast<float>(y), 0.0F, 1.0F});
    }
  }
  cloud.width = cloud.size();
  cloud.height = 1U;
  cloud.is_dense = true;
  return cloud;
}

GlobalKeyframe makeKeyframe(
  std::size_t id, const pcl::PointCloud<pcl::PointXYZI> & scan)
{
  GlobalKeyframe keyframe;
  keyframe.id = id;
  keyframe.registration_scan =
    std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(scan);
  keyframe.occupancy_scan = keyframe.registration_scan;
  keyframe.accumulated_distance = static_cast<double>(id) * 10.0;
  return keyframe;
}

LoopClosureVerifierParameters makeVerifierParameters()
{
  LoopClosureVerifierParameters parameters;
  parameters.submap_neighbor_keyframes = 1U;
  parameters.submap_voxel_leaf_size = 0.10;
  parameters.minimum_overlap_ratio = 0.8;
  parameters.matcher.degeneracy_handling_enabled = false;
  return parameters;
}

TEST(LoopClosureVerifier, AcceptsStructuredGeometryWithSufficientOverlap)
{
  const auto scan = makeStructuredCloud();
  const std::vector<GlobalKeyframe> keyframes{
    makeKeyframe(0U, scan), makeKeyframe(5U, scan)};
  const ScanContextCandidate candidate{0U, 0.0, 0.0, 0.0};

  LoopClosureVerifier verifier(makeVerifierParameters());
  const auto result = verifier.verify(keyframes, 5U, candidate);

  EXPECT_TRUE(result.accepted()) << toString(result.status);
  EXPECT_GE(result.target_points, 100U);
  EXPECT_GT(result.overlap_ratio, 0.8);
  EXPECT_FALSE(result.degenerate);
}

TEST(LoopClosureVerifier, RejectsGroundOnlyCandidateAsDegenerate)
{
  const auto scan = makeGroundCloud();
  const std::vector<GlobalKeyframe> keyframes{
    makeKeyframe(0U, scan), makeKeyframe(5U, scan)};
  const ScanContextCandidate candidate{0U, 0.0, 0.0, 0.0};

  LoopClosureVerifier verifier(makeVerifierParameters());
  const auto result = verifier.verify(keyframes, 5U, candidate);

  EXPECT_EQ(result.status, LoopClosureVerificationStatus::kDegenerateGeometry);
  EXPECT_TRUE(result.degenerate);
}

TEST(LoopClosureVerifier, RejectsFalseCandidateOutsideCorrectionGate)
{
  const auto historical_scan = makeStructuredCloud();
  auto current_scan = historical_scan;
  for (auto & point : current_scan) {
    point.x += 5.0F;
  }
  const std::vector<GlobalKeyframe> keyframes{
    makeKeyframe(0U, historical_scan), makeKeyframe(5U, current_scan)};
  const ScanContextCandidate candidate{0U, 0.0, 0.0, 0.0};

  auto parameters = makeVerifierParameters();
  parameters.matcher.maximum_correction_translation = 0.5;
  LoopClosureVerifier verifier(parameters);
  const auto result = verifier.verify(keyframes, 5U, candidate);

  EXPECT_EQ(result.status, LoopClosureVerificationStatus::kRegistrationRejected);
  EXPECT_FALSE(result.accepted());
}

TEST(LoopClosureVerifier, RejectsGeometricallyMatchedButGloballyInconsistentLoop)
{
  const auto scan = makeStructuredCloud();
  auto historical = makeKeyframe(0U, scan);
  auto current = makeKeyframe(5U, scan);
  current.front_end_base_pose.translation().x() = 20.0;
  const std::vector<GlobalKeyframe> keyframes{historical, current};
  const ScanContextCandidate candidate{0U, 0.0, 0.0, 0.0};

  auto parameters = makeVerifierParameters();
  parameters.maximum_front_end_translation_disagreement = 10.0;
  LoopClosureVerifier verifier(parameters);
  const auto result = verifier.verify(keyframes, 5U, candidate);

  EXPECT_EQ(result.status, LoopClosureVerificationStatus::kFrontEndInconsistent);
  EXPECT_GT(result.front_end_translation_disagreement, 10.0);
}

}  // namespace
}  // namespace slam_robot_slam_3d
