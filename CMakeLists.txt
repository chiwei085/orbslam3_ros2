cmake_minimum_required(VERSION 3.8)
project(orbslam3_ros2 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
include(GNUInstallDirs)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---- C++ standard ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Superbuild
include(${CMAKE_CURRENT_LIST_DIR}/cmake/pangolin_vendor.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/orbslam3_vendor.cmake)

# Ensure colcon build triggers external projects
add_custom_target(vendor ALL DEPENDS pangolin_ep orbslam3_ep)

ament_package()
