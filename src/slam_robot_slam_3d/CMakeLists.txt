cmake_minimum_required(VERSION 3.10)
project(slam_robot_slam_3d)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/grid_contract_check
    scripts/pointcloud_contract_check
    scripts/structured_loop_regression
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
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
endif()

ament_package()
