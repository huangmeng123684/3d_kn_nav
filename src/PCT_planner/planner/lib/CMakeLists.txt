cmake_minimum_required(VERSION 3.13.0)
project(planning_core CXX)

include(CTest)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED true)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=native") # have to enable because of gtsam

set(THIRDPARTY_ROOT "" CACHE PATH "Directory containing GTSAM and OSQP installations")
if(NOT THIRDPARTY_ROOT)
  set(THIRDPARTY_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty)
  set(GTSAM_ROOT ${THIRDPARTY_ROOT}/gtsam-4.1.1/install)
  set(OSQP_ROOT ${THIRDPARTY_ROOT}/osqp/install)
else()
  set(GTSAM_ROOT ${THIRDPARTY_ROOT}/gtsam-4.1.1)
  set(OSQP_ROOT ${THIRDPARTY_ROOT}/osqp)
endif()

set(GTSAM_DIR ${GTSAM_ROOT}/lib/cmake/GTSAM)

find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
  src
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
)

add_subdirectory(src/common)
add_subdirectory(3rdparty/pybind11)
add_subdirectory(src/a_star)
add_subdirectory(src/trajectory_optimization)
add_subdirectory(src/map_manager)
add_subdirectory(src/ele_planner)
