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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree registration)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(point_cloud_preprocessor src/point_cloud_preprocessor.cpp)
target_compile_features(point_cloud_preprocessor PUBLIC cxx_std_17)
target_compile_options(point_cloud_preprocessor PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
  point_cloud_preprocessor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(point_cloud_preprocessor PUBLIC ${PCL_LIBRARIES})

add_library(scan_to_map_matcher src/scan_to_map_matcher.cpp)
target_compile_features(scan_to_map_matcher PUBLIC cxx_std_17)
target_compile_options(scan_to_map_matcher PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
  scan_to_map_matcher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(scan_to_map_matcher PUBLIC ${PCL_LIBRARIES})

add_library(local_submap src/local_submap.cpp)
target_compile_features(local_submap PUBLIC cxx_std_17)
target_compile_options(local_submap PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
  local_submap
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(local_submap PUBLIC ${PCL_LIBRARIES})

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
  local_submap
  scan_to_map_matcher
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
    scripts/front_end_regression
    scripts/grid_contract_check
    scripts/pointcloud_contract_check
    scripts/preprocessing_regression
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
  ament_add_pytest_test(
    test_grid_contract
    test/test_grid_contract.py
  )
  ament_add_pytest_test(
    test_front_end_regression
    test/test_front_end_regression.py
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
    test_height_aware_navigation
    test/test_height_aware_navigation.py
  )
endif()

ament_package()
