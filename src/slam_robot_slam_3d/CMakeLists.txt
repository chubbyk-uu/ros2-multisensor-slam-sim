cmake_minimum_required(VERSION 3.10)
project(slam_robot_slam_3d)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree registration)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(slam_robot_slam REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(point_cloud_preprocessor src/point_cloud_preprocessor.cpp)
target_compile_features(point_cloud_preprocessor PUBLIC cxx_std_17)
target_compile_options(point_cloud_preprocessor PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
  point_cloud_preprocessor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(
  point_cloud_preprocessor
  PUBLIC Eigen3::Eigen ${PCL_LIBRARIES}
)

add_library(scan_to_map_matcher src/scan_to_map_matcher.cpp)
target_compile_features(scan_to_map_matcher PUBLIC cxx_std_17)
target_compile_options(scan_to_map_matcher PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
  scan_to_map_matcher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(scan_to_map_matcher PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})

add_library(local_submap src/local_submap.cpp)
target_compile_features(local_submap PUBLIC cxx_std_17)
target_compile_options(local_submap PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
  local_submap
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(local_submap PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})

add_library(global_keyframe_map src/global_keyframe_map.cpp)
target_compile_features(global_keyframe_map PUBLIC cxx_std_17)
target_compile_options(global_keyframe_map PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
  global_keyframe_map
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(global_keyframe_map rclcpp)
target_link_libraries(global_keyframe_map Eigen3::Eigen ${PCL_LIBRARIES})

add_library(global_point_cloud_map src/global_point_cloud_map.cpp)
target_compile_features(global_point_cloud_map PUBLIC cxx_std_17)
target_compile_options(global_point_cloud_map PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
  global_point_cloud_map
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(
  global_point_cloud_map
  PUBLIC global_keyframe_map Eigen3::Eigen ${PCL_LIBRARIES}
)

add_library(occupancy_projection_contract src/occupancy_projection_contract.cpp)
target_compile_features(occupancy_projection_contract PUBLIC cxx_std_17)
target_compile_options(occupancy_projection_contract PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
  occupancy_projection_contract
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(occupancy_projection_contract PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})

add_library(height_aware_occupancy_grid src/height_aware_occupancy_grid.cpp)
target_compile_features(height_aware_occupancy_grid PUBLIC cxx_std_17)
target_compile_options(height_aware_occupancy_grid PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
  height_aware_occupancy_grid
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  height_aware_occupancy_grid
  PUBLIC global_keyframe_map occupancy_projection_contract
  slam_robot_slam::occupancy_grid_map Eigen3::Eigen
)

add_library(slam_snapshot src/slam_snapshot.cpp)
target_compile_features(slam_snapshot PUBLIC cxx_std_17)
target_compile_options(slam_snapshot PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(slam_snapshot PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(
  slam_snapshot
  PUBLIC global_keyframe_map occupancy_projection_contract Eigen3::Eigen ${PCL_LIBRARIES}
)

add_library(scan_context_index src/scan_context_index.cpp)
target_compile_features(scan_context_index PUBLIC cxx_std_17)
target_compile_options(scan_context_index PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
  scan_context_index
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(
  scan_context_index
  PUBLIC global_keyframe_map Eigen3::Eigen ${PCL_LIBRARIES}
)

add_library(loop_closure_verifier src/loop_closure_verifier.cpp)
target_compile_features(loop_closure_verifier PUBLIC cxx_std_17)
target_compile_options(loop_closure_verifier PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
  loop_closure_verifier
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${PCL_INCLUDE_DIRS}
)

add_library(se2_pose_graph_backend src/se2_pose_graph_backend.cpp)
target_compile_features(se2_pose_graph_backend PUBLIC cxx_std_17)
target_compile_options(se2_pose_graph_backend PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
  se2_pose_graph_backend
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  se2_pose_graph_backend
  PUBLIC global_keyframe_map slam_robot_slam::pose_graph_2d Eigen3::Eigen
)

add_library(
  pose_graph_submission_state
  src/pose_graph_submission_state.cpp
)
target_compile_features(pose_graph_submission_state PUBLIC cxx_std_17)
target_compile_options(
  pose_graph_submission_state PRIVATE -Wall -Wextra -Wpedantic
)
target_include_directories(
  pose_graph_submission_state
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(pose_graph_submission_state PUBLIC se2_pose_graph_backend)
target_link_libraries(
  loop_closure_verifier
  PUBLIC global_keyframe_map scan_context_index scan_to_map_matcher
  Eigen3::Eigen ${PCL_LIBRARIES}
)

add_library(odometry_interpolator src/odometry_interpolator.cpp)
target_compile_features(odometry_interpolator PUBLIC cxx_std_17)
target_compile_options(odometry_interpolator PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
  odometry_interpolator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(odometry_interpolator nav_msgs rclcpp)
target_link_libraries(odometry_interpolator Eigen3::Eigen)

add_library(match_failure_recovery src/match_failure_recovery.cpp)
target_compile_features(match_failure_recovery PUBLIC cxx_std_17)
target_compile_options(match_failure_recovery PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
  match_failure_recovery
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(match_failure_recovery PUBLIC ${PCL_LIBRARIES})

add_executable(
  point_cloud_preprocessor_node
  src/point_cloud_preprocessor_node.cpp
)
target_compile_features(point_cloud_preprocessor_node PUBLIC cxx_std_17)
target_compile_options(
  point_cloud_preprocessor_node PRIVATE -Wall -Wextra -Wpedantic
)
target_link_libraries(point_cloud_preprocessor_node point_cloud_preprocessor)
ament_target_dependencies(
  point_cloud_preprocessor_node
  diagnostic_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
)

add_executable(scan_to_map_odometry_node src/scan_to_map_odometry_node.cpp)
target_compile_features(scan_to_map_odometry_node PUBLIC cxx_std_17)
target_compile_options(
  scan_to_map_odometry_node PRIVATE -Wall -Wextra -Wpedantic
)
target_link_libraries(
  scan_to_map_odometry_node
  point_cloud_preprocessor
  local_submap
  global_keyframe_map
  global_point_cloud_map
  height_aware_occupancy_grid
  loop_closure_verifier
  se2_pose_graph_backend
  pose_graph_submission_state
  scan_context_index
  match_failure_recovery
  odometry_interpolator
  scan_to_map_matcher
  slam_snapshot
  ${PCL_LIBRARIES}
)
ament_target_dependencies(
  scan_to_map_odometry_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_srvs
  tf2_ros
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS point_cloud_preprocessor_node scan_to_map_odometry_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/dataset_contract_check
    scripts/corridor_3d_regression
    scripts/exploration_campaign
    scripts/front_end_motion_regression
    scripts/front_end_regression
    scripts/frontier_exploration_regression
    scripts/frontier_selection_replay
    scripts/grid_contract_check
    scripts/launch_smoke_check
    scripts/map_projection_census
    scripts/map_projection_compare
    scripts/nav2_fault_injection_regression
    scripts/pointcloud_contract_check
    scripts/preprocessing_regression
    scripts/recorded_odom_tf_publisher
    scripts/recorded_static_tf_publisher
    scripts/rgbd_dataset_contract_check
    scripts/rtabmap_rgbd_fixed_regression_check
    scripts/snapshot_localization_regression
    scripts/snapshot_resume_probe
    scripts/snapshot_resume_regression
    scripts/stack_comparison_distribution
    scripts/stack_comparison_report
    scripts/stack_trajectory_census
    scripts/structured_loop_regression
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(
    ament_cmake_flake8_CONFIG_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/setup.cfg"
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_dataset_contract
    test/test_dataset_contract.py
  )
  ament_add_pytest_test(
    test_rgbd_dataset_contract
    test/test_rgbd_dataset_contract.py
  )
  ament_add_pytest_test(
    test_rgbd_dataset_launch
    test/test_rgbd_dataset_launch.py
  )
  ament_add_gtest(
    test_point_cloud_preprocessor
    test/test_point_cloud_preprocessor.cpp
  )
  target_link_libraries(
    test_point_cloud_preprocessor
    point_cloud_preprocessor
  )
  ament_add_gtest(
    test_scan_to_map_matcher
    test/test_scan_to_map_matcher.cpp
  )
  target_link_libraries(
    test_scan_to_map_matcher
    scan_to_map_matcher
  )
  ament_add_gtest(
    test_local_submap
    test/test_local_submap.cpp
  )
  target_link_libraries(
    test_local_submap
    local_submap
  )
  ament_add_gtest(
    test_global_keyframe_map
    test/test_global_keyframe_map.cpp
  )
  target_link_libraries(
    test_global_keyframe_map
    global_keyframe_map
  )
  ament_add_gtest(
    test_global_map_pose_selector
    test/test_global_map_pose_selector.cpp
  )
  target_link_libraries(test_global_map_pose_selector global_keyframe_map)
  ament_add_gtest(
    test_global_point_cloud_map
    test/test_global_point_cloud_map.cpp
  )
  target_link_libraries(
    test_global_point_cloud_map
    global_point_cloud_map
  )
  ament_add_gtest(
    test_height_aware_occupancy_grid
    test/test_height_aware_occupancy_grid.cpp
  )
  target_link_libraries(
    test_height_aware_occupancy_grid
    height_aware_occupancy_grid
  )
  ament_add_gtest(test_slam_snapshot test/test_slam_snapshot.cpp)
  target_link_libraries(test_slam_snapshot slam_snapshot)
  ament_add_gtest(
    test_scan_context_index
    test/test_scan_context_index.cpp
  )
  target_link_libraries(
    test_scan_context_index
    scan_context_index
  )
  ament_add_gtest(
    test_loop_closure_verifier
    test/test_loop_closure_verifier.cpp
  )
  target_link_libraries(
    test_loop_closure_verifier
    loop_closure_verifier
  )
  ament_add_gtest(
    test_se2_pose_graph_backend
    test/test_se2_pose_graph_backend.cpp
  )
  target_link_libraries(
    test_se2_pose_graph_backend
    se2_pose_graph_backend
  )
  ament_add_gtest(
    test_pose_graph_submission_state
    test/test_pose_graph_submission_state.cpp
  )
  target_link_libraries(
    test_pose_graph_submission_state
    pose_graph_submission_state
  )
  ament_add_gtest(
    test_odometry_interpolator
    test/test_odometry_interpolator.cpp
  )
  target_link_libraries(
    test_odometry_interpolator
    odometry_interpolator
  )
  ament_add_gtest(
    test_match_failure_recovery
    test/test_match_failure_recovery.cpp
  )
  target_link_libraries(
    test_match_failure_recovery
    match_failure_recovery
  )
  ament_add_pytest_test(
    test_grid_contract
    test/test_grid_contract.py
  )
  ament_add_pytest_test(
    test_map_projection_census
    test/test_map_projection_census.py
  )
  ament_add_pytest_test(
    test_stack_comparison
    test/test_stack_comparison.py
  )
  ament_add_pytest_test(
    test_exploration_campaign
    test/test_exploration_campaign.py
  )
  ament_add_pytest_test(
    test_nav2_fault_injection_regression
    test/test_nav2_fault_injection_regression.py
  )
  ament_add_pytest_test(
    test_recorded_odom_tf_publisher
    test/test_recorded_odom_tf_publisher.py
  )
  ament_add_pytest_test(
    test_front_end_regression
    test/test_front_end_regression.py
  )
  ament_add_pytest_test(
    test_front_end_motion_regression
    test/test_front_end_motion_regression.py
  )
  ament_add_pytest_test(
    test_corridor_3d_regression
    test/test_corridor_3d_regression.py
  )
  ament_add_pytest_test(
    test_pointcloud_contract
    test/test_pointcloud_contract.py
  )
  ament_add_pytest_test(
    test_structured_loop_regression
    test/test_structured_loop_regression.py
  )
  ament_add_pytest_test(
    test_rtabmap_config
    test/test_rtabmap_config.py
  )
  ament_add_pytest_test(
    test_rtabmap_rgbd_config
    test/test_rtabmap_rgbd_config.py
  )
  ament_add_pytest_test(
    test_rtabmap_rgbd_fixed_launch
    test/test_rtabmap_rgbd_fixed_launch.py
  )
  ament_add_pytest_test(
    test_rtabmap_rgbd_fixed_regression
    test/test_rtabmap_rgbd_fixed_regression.py
  )
  ament_add_pytest_test(
    test_rtabmap_rgbd_online_launch
    test/test_rtabmap_rgbd_online_launch.py
  )
  ament_add_pytest_test(
    test_rtabmap_rgbd_navigation_launch
    test/test_rtabmap_rgbd_navigation_launch.py
  )
  ament_add_pytest_test(
    test_large_cloud_transport
    test/test_large_cloud_transport.py
  )
  ament_add_pytest_test(
    test_test_isolation
    test/test_test_isolation.py
  )
  ament_add_pytest_test(
    test_height_aware_navigation
    test/test_height_aware_navigation.py
  )
  ament_add_pytest_test(
    test_launch_smoke_check
    test/test_launch_smoke_check.py
  )
  ament_add_pytest_test(
    test_navigation_safety_launch
    test/test_navigation_safety_launch.py
  )
  ament_add_pytest_test(
    test_frontier_exploration_regression
    test/test_frontier_exploration_regression.py
  )
  ament_add_pytest_test(
    test_snapshot_resume_regression
    test/test_snapshot_resume_regression.py
  )
  ament_add_pytest_test(
    test_snapshot_localization_regression
    test/test_snapshot_localization_regression.py
  )
  ament_add_pytest_test(
    test_trinary_projection_contract
    test/test_trinary_projection_contract.py
  )
  ament_add_pytest_test(
    test_exploration_seed_wiring
    test/test_exploration_seed_wiring.py
  )
  ament_add_pytest_test(
    test_frontier_selection_replay
    test/test_frontier_selection_replay.py
  )
endif()

ament_package()
