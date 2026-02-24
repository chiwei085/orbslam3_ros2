cmake_minimum_required(VERSION 3.8)
project(orbslam3_ros2 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_srvs REQUIRED)
include(GNUInstallDirs)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---- C++ standard ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Superbuild
include(${CMAKE_CURRENT_LIST_DIR}/cmake/pangolin_vendor.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/orbslam3_vendor.cmake)
find_package(Pangolin REQUIRED)

# Ensure colcon build triggers external projects
add_custom_target(vendor ALL DEPENDS pangolin_ep orbslam3_ep)

add_executable(ros_rgbd
  src/ros_rgbd.cpp
)

add_dependencies(ros_rgbd vendor)

target_include_directories(ros_rgbd PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ORB_SLAM3
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ORB_SLAM3/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ORB_SLAM3/include/CameraModels
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ORB_SLAM3/Thirdparty/Sophus
)

ament_target_dependencies(ros_rgbd
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  message_filters
  cv_bridge
  image_transport
  std_srvs
)

target_link_libraries(ros_rgbd
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/libORB_SLAM3.so
)

install(TARGETS ros_rgbd
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/ config/
  DESTINATION share/${PROJECT_NAME}
)

install(FILES third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt
  DESTINATION share/${PROJECT_NAME}/Vocabulary
)

ament_package()
