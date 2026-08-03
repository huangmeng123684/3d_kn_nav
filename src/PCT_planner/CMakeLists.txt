cmake_minimum_required(VERSION 3.8)
project(pct_planner)

find_package(ament_cmake REQUIRED)

# Keep the ROS 2 executable name unchanged while installing the Python node directly.
install(PROGRAMS
  scripts/run_ros2_global_planner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME run_ros2_global_planner
)

# Install parameter files
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_planner_wrapper_short_path
    test/test_planner_wrapper_short_path.py
  )
endif()

ament_package()
