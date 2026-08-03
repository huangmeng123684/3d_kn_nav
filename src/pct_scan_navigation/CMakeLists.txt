cmake_minimum_required(VERSION 3.8)
project(pct_scan_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LocalizationStatus.msg"
  "msg/NavigationStatus.msg"
  "msg/MapStatus.msg"
  "srv/SwitchMap.srv"
  "srv/RestartNavigation.srv"
  "srv/LoadLocalizationMap.srv"
  "srv/LoadTomogram.srv"
  DEPENDENCIES std_msgs
)

add_library(waypoint_utils src/waypoint_utils.cpp)
target_include_directories(waypoint_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(waypoint_utils geometry_msgs nav_msgs)

add_executable(pct_scan_coordinator src/pct_scan_coordinator.cpp)
target_include_directories(pct_scan_coordinator PRIVATE include)
ament_target_dependencies(pct_scan_coordinator
  geometry_msgs nav_msgs rclcpp std_srvs)
target_link_libraries(pct_scan_coordinator waypoint_utils)

install(TARGETS pct_scan_coordinator waypoint_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS
  scripts/global_path_follow_coordinator_node.py
  scripts/nav_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/global_path_follow_coordinator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pct_global_path_follow_coordinator
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_waypoint_utils test/test_waypoint_utils.cpp)
  target_link_libraries(test_waypoint_utils waypoint_utils)
  ament_add_pytest_test(test_navigation_contract test/test_navigation_contract.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
