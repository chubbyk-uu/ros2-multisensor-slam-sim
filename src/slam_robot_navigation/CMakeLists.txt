cmake_minimum_required(VERSION 3.10)
project(slam_robot_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(frontier_detector src/frontier_detector.cpp)
target_compile_features(frontier_detector PUBLIC cxx_std_17)
target_compile_options(frontier_detector PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(frontier_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(frontier_detector nav_msgs)

add_library(frontier_goal_selector src/frontier_goal_selector.cpp)
target_compile_features(frontier_goal_selector PUBLIC cxx_std_17)
target_compile_options(frontier_goal_selector PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(frontier_goal_selector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(frontier_goal_selector frontier_detector)

add_executable(frontier_explorer_node src/frontier_explorer_node.cpp)
target_compile_features(frontier_explorer_node PUBLIC cxx_std_17)
target_compile_options(frontier_explorer_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  frontier_explorer_node
  frontier_detector
  frontier_goal_selector
)
ament_target_dependencies(frontier_explorer_node
  diagnostic_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  tf2
  tf2_ros
  visualization_msgs
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS frontier_explorer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Installed as an importable package rather than a script so the 3D
# regression can reach the same blocked-road criteria; two copies of a safety
# criterion would be free to drift apart unnoticed.
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS scripts/navigation_regression.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_frontier_detector test/test_frontier_detector.cpp)
  target_link_libraries(test_frontier_detector frontier_detector)
  ament_add_gtest(
    test_frontier_goal_selector
    test/test_frontier_goal_selector.cpp
  )
  target_link_libraries(
    test_frontier_goal_selector
    frontier_goal_selector
  )
  ament_add_gtest(
    test_frontier_action_deadline
    test/test_frontier_action_deadline.cpp
  )
  target_include_directories(
    test_frontier_action_deadline
    PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_add_gtest(
    test_empty_frontier_counter
    test/test_empty_frontier_counter.cpp
  )
  target_include_directories(
    test_empty_frontier_counter
    PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_add_gtest(
    test_navigation_availability
    test/test_navigation_availability.cpp
  )
  target_include_directories(
    test_navigation_availability
    PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_pytest_test(test_blocked_road test/test_blocked_road.py)
endif()

ament_package()
