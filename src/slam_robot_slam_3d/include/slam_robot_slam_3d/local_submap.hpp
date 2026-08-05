#ifndef SLAM_ROBOT_SLAM_3D__LOCAL_SUBMAP_HPP_
#define SLAM_ROBOT_SLAM_3D__LOCAL_SUBMAP_HPP_

#include <cstddef>
#include <deque>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace slam_robot_slam_3d
{

struct LocalSubmapParameters
{
  std::size_t maximum_keyframes{12U};
  double voxel_leaf_size{0.15};
};

class LocalSubmap
{
public:
  explicit LocalSubmap(LocalSubmapParameters parameters);

  void addKeyframe(
    const pcl::PointCloud<pcl::PointXYZI> & scan,
    const Eigen::Isometry3d & scan_pose);
  void clear();

  const pcl::PointCloud<pcl::PointXYZI> & cloud() const;
  std::size_t keyframeCount() const;
  const LocalSubmapParameters & parameters() const;

private:
  void rebuild();
  void validateParameters() const;

  LocalSubmapParameters parameters_;
  std::deque<pcl::PointCloud<pcl::PointXYZI>> keyframes_;
  pcl::PointCloud<pcl::PointXYZI> cloud_;
};

}  // namespace slam_robot_slam_3d

#endif  // SLAM_ROBOT_SLAM_3D__LOCAL_SUBMAP_HPP_
