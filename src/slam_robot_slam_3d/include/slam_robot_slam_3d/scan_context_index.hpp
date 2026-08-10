#ifndef SLAM_ROBOT_SLAM_3D__SCAN_CONTEXT_INDEX_HPP_
#define SLAM_ROBOT_SLAM_3D__SCAN_CONTEXT_INDEX_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "slam_robot_slam_3d/global_keyframe_map.hpp"

namespace slam_robot_slam_3d
{

struct ScanContextParameters
{
  double maximum_radius{20.0};
  std::size_t radial_bins{20U};
  std::size_t angular_bins{60U};
  std::size_t minimum_keyframe_separation{80U};
  double minimum_travel_distance{8.0};
  // Retrieval is only a proposal stage. Do not hand arbitrarily dissimilar
  // places to geometric verification merely because they rank first.
  double maximum_descriptor_distance{0.15};
  std::size_t ring_key_candidate_count{20U};
  std::size_t maximum_candidates{5U};
};

struct ScanContextCandidate
{
  std::size_t keyframe_id{0U};
  double descriptor_distance{0.0};
  double ring_key_distance{0.0};
  double predicted_yaw{0.0};
};

// Incremental Scan Context-style retrieval index. It ranks visual-place
// candidates only; geometric verification decides whether any candidate is a
// loop closure in the next implementation stage.
class ScanContextIndex
{
public:
  explicit ScanContextIndex(ScanContextParameters parameters);

  std::vector<ScanContextCandidate> addAndQuery(
    const GlobalKeyframe & keyframe);
  std::size_t size() const;

private:
  struct Descriptor
  {
    Eigen::MatrixXf cells;
    Eigen::VectorXf ring_key;
  };

  struct Entry
  {
    std::size_t keyframe_id{0U};
    double accumulated_distance{0.0};
    Descriptor descriptor;
  };

  Descriptor makeDescriptor(const GlobalKeyframe & keyframe) const;
  std::vector<ScanContextCandidate> query(const Entry & current) const;
  void validateParameters() const;

  ScanContextParameters parameters_;
  std::vector<Entry> entries_;
};

}  // namespace slam_robot_slam_3d

#endif  // SLAM_ROBOT_SLAM_3D__SCAN_CONTEXT_INDEX_HPP_
