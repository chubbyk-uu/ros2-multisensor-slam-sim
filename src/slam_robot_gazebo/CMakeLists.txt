cmake_minimum_required(VERSION 3.10)
project(slam_robot_gazebo)

find_package(ament_cmake REQUIRED)
find_package(gz-common5 REQUIRED COMPONENTS graphics)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sdformat14 REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(safe_spawn_sampler_core
  src/safe_spawn_sampler.cpp
  src/sdf_world_geometry.cpp
  src/spawn_record.cpp
)
target_include_directories(safe_spawn_sampler_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(safe_spawn_sampler_core
  gz-common5::graphics
  nlohmann_json::nlohmann_json
  sdformat14::sdformat14
)
target_compile_options(safe_spawn_sampler_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(safe_spawn_sampler src/safe_spawn_sampler_main.cpp)
target_link_libraries(safe_spawn_sampler safe_spawn_sampler_core)
target_compile_options(safe_spawn_sampler PRIVATE -Wall -Wextra -Wpedantic)

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
  TARGETS safe_spawn_sampler sensor_covariance_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_safe_spawn_sampler test/test_safe_spawn_sampler.cpp)
  target_link_libraries(test_safe_spawn_sampler safe_spawn_sampler_core)
  target_compile_definitions(test_safe_spawn_sampler PRIVATE
    TEST_WORLD_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/worlds")
  ament_add_pytest_test(
    test_simulation_launch
    test/test_simulation_launch.py
  )
endif()

ament_package()
