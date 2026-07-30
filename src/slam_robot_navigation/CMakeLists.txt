cmake_minimum_required(VERSION 3.8)
project(slam_robot_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/navigation_regression.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
