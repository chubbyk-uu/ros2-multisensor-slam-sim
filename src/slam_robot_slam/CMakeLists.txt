cmake_minimum_required(VERSION 3.8)
project(slam_robot_slam)

find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/save_slam_map
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
