#ifndef SLAM_ROBOT_SLAM__LATEST_SNAPSHOT_QUEUE_HPP_
#define SLAM_ROBOT_SLAM__LATEST_SNAPSHOT_QUEUE_HPP_

#include <optional>
#include <utility>

namespace slam_robot_slam
{

template<typename SnapshotT>
class LatestSnapshotQueue
{
public:
  void request(SnapshotT snapshot)
  {
    if (!active_.has_value()) {
      active_ = std::move(snapshot);
      return;
    }
    queued_ = std::move(snapshot);
  }

  bool hasActive() const
  {
    return active_.has_value();
  }

  bool hasQueued() const
  {
    return queued_.has_value();
  }

  SnapshotT & active()
  {
    return active_.value();
  }

  const SnapshotT & active() const
  {
    return active_.value();
  }

  const SnapshotT & queued() const
  {
    return queued_.value();
  }

  bool completeActive()
  {
    active_.reset();
    if (!queued_.has_value()) {
      return false;
    }
    active_ = std::move(queued_);
    queued_.reset();
    return true;
  }

  void clear()
  {
    active_.reset();
    queued_.reset();
  }

private:
  std::optional<SnapshotT> active_;
  std::optional<SnapshotT> queued_;
};

}  // namespace slam_robot_slam

#endif  // SLAM_ROBOT_SLAM__LATEST_SNAPSHOT_QUEUE_HPP_
