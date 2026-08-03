cmake_minimum_required(VERSION 3.10)
project(slam_robot_gazebo)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(sensor_covariance_adapter_node
  src/sensor_covariance_adapter_node.cpp
)
ament_target_dependencies(
  sensor_covariance_adapter_node
  nav_msgs
  rclcpp
  sensor_msgs
)
target_compile_options(
  sensor_covariance_adapter_node
  PRIVATE -Wall -Wextra -Wpedantic
)

install(
  DIRECTORY config launch rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/imu_fusion_regression
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS sensor_covariance_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
