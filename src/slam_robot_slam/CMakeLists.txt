cmake_minimum_required(VERSION 3.8)
project(slam_robot_slam)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

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

add_executable(laser_scan_preprocessor_node
  src/laser_scan_preprocessor_node.cpp
)
target_link_libraries(laser_scan_preprocessor_node laser_scan_preprocessor)
ament_target_dependencies(laser_scan_preprocessor_node rclcpp sensor_msgs)
target_compile_options(
  laser_scan_preprocessor_node
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
  TARGETS laser_scan_preprocessor laser_scan_preprocessor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  PROGRAMS scripts/save_slam_map
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    test_laser_scan_preprocessor
    test/test_laser_scan_preprocessor.cpp
  )
  target_link_libraries(
    test_laser_scan_preprocessor
    laser_scan_preprocessor
  )
  ament_target_dependencies(test_laser_scan_preprocessor sensor_msgs)
endif()

ament_package()
