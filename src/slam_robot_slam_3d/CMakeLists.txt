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
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

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
  TARGETS point_cloud_preprocessor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/dataset_contract_check
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
  ament_add_pytest_test(
    test_grid_contract
    test/test_grid_contract.py
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
