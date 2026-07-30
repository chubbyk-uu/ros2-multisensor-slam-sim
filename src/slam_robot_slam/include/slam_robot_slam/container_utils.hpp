#ifndef SLAM_ROBOT_SLAM__CONTAINER_UTILS_HPP_
#define SLAM_ROBOT_SLAM__CONTAINER_UTILS_HPP_

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace slam_robot_slam
{

template<typename ValueT>
void trimOldestInBatches(
  std::vector<ValueT> & values,
  const std::size_t maximum_size)
{
  if (maximum_size == 0U) {
    throw std::invalid_argument("Maximum container size must be positive");
  }
  if (values.size() <= maximum_size) {
    return;
  }

  const std::size_t batch_size =
    std::max(maximum_size / std::size_t{10U}, std::size_t{1U});
  const std::size_t retained_size =
    maximum_size > batch_size ? maximum_size - batch_size : maximum_size;
  const std::size_t erase_count = values.size() - retained_size;
  values.erase(
    values.begin(),
    values.begin() + static_cast<std::ptrdiff_t>(erase_count));
}

}  // namespace slam_robot_slam

#endif  // SLAM_ROBOT_SLAM__CONTAINER_UTILS_HPP_
