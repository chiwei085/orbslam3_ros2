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
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Pangolin CONFIG REQUIRED)
include(GNUInstallDirs)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ORBROS_SHUTDOWN_TRACE "Enable shutdown tracing logs in wrapper and ORB_SLAM3" OFF)

# ---- C++ standard ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(ORBROS_SHUTDOWN_TRACE)
  add_compile_definitions(ORBROS_SHUTDOWN_TRACE=1)
endif()

find_package(ORB_SLAM3 CONFIG REQUIRED)

add_executable(ros_rgbd
  src/ros_rgbd.cpp
)

add_executable(orbslam3_export_osa
  src/export_osa.cpp
)

target_include_directories(ros_rgbd PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

target_include_directories(orbslam3_export_osa PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(ros_rgbd
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  cv_bridge
  image_transport
  std_srvs
)

target_link_libraries(ros_rgbd
  ${OpenCV_LIBS}
  Pangolin::pangolin
  ORB_SLAM3::ORB_SLAM3
)

target_link_libraries(orbslam3_export_osa
  ${OpenCV_LIBS}
  Pangolin::pangolin
  ORB_SLAM3::ORB_SLAM3
)

install(TARGETS ros_rgbd orbslam3_export_osa
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/ config/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_viewer_backend_policy
    test/test_viewer_backend_policy.cpp
  )
  if(TARGET test_viewer_backend_policy)
    target_include_directories(test_viewer_backend_policy PRIVATE
      include
    )
    set_tests_properties(test_viewer_backend_policy PROPERTIES
      ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0"
    )
  endif()
endif()

set(ORBROS_ORBVOC_FILE
  "${CMAKE_CURRENT_SOURCE_DIR}/../orbslam3_ros2_vendor/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt"
)
if(NOT EXISTS "${ORBROS_ORBVOC_FILE}")
  message(FATAL_ERROR
    "ORBvoc.txt not found at ${ORBROS_ORBVOC_FILE}. "
    "Extract Vocabulary/ORBvoc.txt.tar.gz in src/orbslam3_ros2_vendor/third_party/ORB_SLAM3/Vocabulary/ first."
  )
endif()

install(FILES "${ORBROS_ORBVOC_FILE}"
  DESTINATION share/${PROJECT_NAME}/Vocabulary
)

ament_package()
