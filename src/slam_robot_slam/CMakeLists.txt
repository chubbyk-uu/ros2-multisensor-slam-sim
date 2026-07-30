cmake_minimum_required(VERSION 3.10)
project(slam_robot_slam)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(
    CMAKE_BUILD_TYPE RelWithDebInfo
    CACHE STRING "Choose the build type." FORCE
  )
  set_property(
    CACHE CMAKE_BUILD_TYPE
    PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Ceres REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(laser_scan_preprocessor
  src/laser_scan_preprocessor.cpp
)
target_include_directories(laser_scan_preprocessor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(laser_scan_preprocessor sensor_msgs)
target_compile_options(laser_scan_preprocessor PRIVATE -Wall -Wextra -Wpedantic)

add_library(pose2d
  src/pose2d.cpp
)
target_include_directories(pose2d
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(pose2d PRIVATE -Wall -Wextra -Wpedantic)

add_library(point_to_line_icp
  src/point_to_line_icp.cpp
)
target_include_directories(point_to_line_icp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(point_to_line_icp pose2d)
target_compile_options(point_to_line_icp PRIVATE -Wall -Wextra -Wpedantic)

add_library(correlative_scan_matcher
  src/correlative_scan_matcher.cpp
)
target_include_directories(correlative_scan_matcher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  correlative_scan_matcher
  pose2d
)
target_compile_options(
  correlative_scan_matcher
  PRIVATE -Wall -Wextra -Wpedantic
)

add_library(occupancy_grid_map
  src/occupancy_grid_map.cpp
)
target_include_directories(occupancy_grid_map
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(
  occupancy_grid_map
  PRIVATE -Wall -Wextra -Wpedantic
)

add_library(pose_graph_2d
  src/pose_graph_2d.cpp
)
target_include_directories(pose_graph_2d
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(pose_graph_2d pose2d Ceres::ceres)
target_compile_options(
  pose_graph_2d
  PRIVATE -Wall -Wextra -Wpedantic
)

add_library(loop_closure_detector
  src/loop_closure_detector.cpp
)
target_include_directories(loop_closure_detector
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(loop_closure_detector pose2d)
target_compile_options(
  loop_closure_detector
  PRIVATE -Wall -Wextra -Wpedantic
)

add_library(loop_closure_processor
  src/loop_closure_processor.cpp
)
target_include_directories(loop_closure_processor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  loop_closure_processor
  correlative_scan_matcher
  loop_closure_detector
  pose2d
  pose_graph_2d
)
target_compile_options(
  loop_closure_processor
  PRIVATE -Wall -Wextra -Wpedantic
)

add_executable(laser_scan_preprocessor_node
  src/laser_scan_preprocessor_node.cpp
)
target_link_libraries(laser_scan_preprocessor_node laser_scan_preprocessor)
ament_target_dependencies(laser_scan_preprocessor_node rclcpp sensor_msgs)
target_compile_options(
  laser_scan_preprocessor_node
  PRIVATE -Wall -Wextra -Wpedantic
)

add_executable(scan_matcher_odometry_node
  src/scan_matcher_odometry_node.cpp
)
target_link_libraries(
  scan_matcher_odometry_node
  correlative_scan_matcher
  laser_scan_preprocessor
  occupancy_grid_map
  pose2d
  pose_graph_2d
  loop_closure_detector
  loop_closure_processor
)
ament_target_dependencies(
  scan_matcher_odometry_node
  builtin_interfaces
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)
target_compile_options(
  scan_matcher_odometry_node
  PRIVATE -Wall -Wextra -Wpedantic
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    laser_scan_preprocessor
    laser_scan_preprocessor_node
    loop_closure_detector
    loop_closure_processor
    correlative_scan_matcher
    occupancy_grid_map
    point_to_line_icp
    pose2d
    pose_graph_2d
    scan_matcher_odometry_node
  EXPORT export_slam_robot_slam
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_targets(export_slam_robot_slam HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Ceres sensor_msgs)

install(
  PROGRAMS
    scripts/save_slam_map
    scripts/scan_matcher_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(
    test_laser_scan_preprocessor
    test/test_laser_scan_preprocessor.cpp
  )
  target_link_libraries(
    test_laser_scan_preprocessor
    laser_scan_preprocessor
  )
  ament_target_dependencies(test_laser_scan_preprocessor sensor_msgs)

  ament_add_gtest(
    test_point_to_line_icp
    test/test_point_to_line_icp.cpp
  )
  target_link_libraries(
    test_point_to_line_icp
    laser_scan_preprocessor
    point_to_line_icp
  )

  ament_add_gtest(
    test_pose2d
    test/test_pose2d.cpp
  )
  target_link_libraries(test_pose2d pose2d)

  ament_add_gtest(
    test_correlative_scan_matcher
    test/test_correlative_scan_matcher.cpp
  )
  target_link_libraries(
    test_correlative_scan_matcher
    correlative_scan_matcher
    laser_scan_preprocessor
    pose2d
  )

  ament_add_gtest(
    test_occupancy_grid_map
    test/test_occupancy_grid_map.cpp
  )
  target_link_libraries(
    test_occupancy_grid_map
    laser_scan_preprocessor
    occupancy_grid_map
  )

  ament_add_gtest(
    test_pose_graph_2d
    test/test_pose_graph_2d.cpp
  )
  target_link_libraries(
    test_pose_graph_2d
    pose2d
    pose_graph_2d
  )

  ament_add_gtest(
    test_loop_closure_detector
    test/test_loop_closure_detector.cpp
  )
  target_link_libraries(
    test_loop_closure_detector
    loop_closure_detector
    pose2d
  )

  ament_add_gtest(
    test_loop_closure_processor
    test/test_loop_closure_processor.cpp
  )
  target_link_libraries(
    test_loop_closure_processor
    loop_closure_processor
  )

  ament_add_gtest(
    test_latest_snapshot_queue
    test/test_latest_snapshot_queue.cpp
  )
  target_include_directories(
    test_latest_snapshot_queue
    PRIVATE include
  )
endif()

ament_package()
