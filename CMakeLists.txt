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

option(ORBROS_ENABLE_ASAN "Enable AddressSanitizer for wrapper and vendored deps" OFF)
option(ORBROS_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer for wrapper and vendored deps" OFF)
option(ORBROS_ENABLE_ASAN_UBSAN "Legacy switch to enable both ASAN and UBSAN" OFF)
option(ORBROS_SHUTDOWN_TRACE "Enable shutdown tracing logs in wrapper and ORB_SLAM3" OFF)

set(ORBROS_SANITIZER_FLAGS "" CACHE STRING "Extra sanitizer compile/link flags propagated to wrapper and vendored deps")

# ---- C++ standard ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(ORBROS_ENABLE_ASAN_UBSAN)
  set(ORBROS_ENABLE_ASAN ON)
  set(ORBROS_ENABLE_UBSAN ON)
endif()

if((ORBROS_ENABLE_ASAN OR ORBROS_ENABLE_UBSAN) AND NOT ORBROS_SANITIZER_FLAGS)
  set(_orbros_sanitize_kinds "")
  if(ORBROS_ENABLE_ASAN)
    list(APPEND _orbros_sanitize_kinds "address")
  endif()
  if(ORBROS_ENABLE_UBSAN)
    list(APPEND _orbros_sanitize_kinds "undefined")
  endif()
  list(JOIN _orbros_sanitize_kinds "," _orbros_sanitize_csv)
  set(ORBROS_SANITIZER_FLAGS "-fsanitize=${_orbros_sanitize_csv} -O1 -g -fno-omit-frame-pointer")
endif()

if(ORBROS_SANITIZER_FLAGS)
  separate_arguments(ORBROS_SANITIZER_FLAGS_LIST NATIVE_COMMAND "${ORBROS_SANITIZER_FLAGS}")
  add_compile_options(${ORBROS_SANITIZER_FLAGS_LIST})
  add_link_options(${ORBROS_SANITIZER_FLAGS_LIST})
endif()

if(ORBROS_SHUTDOWN_TRACE)
  add_compile_definitions(ORBROS_SHUTDOWN_TRACE=1)
endif()

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

install(FILES third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt
  DESTINATION share/${PROJECT_NAME}/Vocabulary
)

ament_package()
