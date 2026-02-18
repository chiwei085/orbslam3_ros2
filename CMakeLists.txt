cmake_minimum_required(VERSION 3.8)
project(orbslam3_ros2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(ORBSLAM3_EIGEN_SAFE_MODE "Disable Eigen vectorization/alignment-sensitive paths for diagnostics" OFF)
option(ORBSLAM3_DISABLE_FAST_MATH "Force-disable fast-math on local ORB-SLAM3 targets for diagnostics" ON)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Conan deps in this workspace are generated as Release artifacts. Force
# single-config generators (Ninja/Makefiles) to use Release to keep imported
# target locations consistent.
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE OR NOT CMAKE_BUILD_TYPE STREQUAL "Release")
        set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    endif()
endif()

# Auto-discover Conan CMakeDeps output from workspace root so plain colcon
# builds can resolve Pangolin/Sophus/g2o/DBoW2 without extra cmake args.
set(_ORB_CONAN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../build/conan")
if(EXISTS "${_ORB_CONAN_DIR}")
    list(APPEND CMAKE_PREFIX_PATH "${_ORB_CONAN_DIR}")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(OpenCV REQUIRED)
find_package(JPEG QUIET)
find_package(Eigen3 REQUIRED CONFIG)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(g2o REQUIRED)
find_package(DBoW2 REQUIRED)
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

# Resolve OpenCV link target across Conan CMakeDeps and system OpenCV packages.
if(TARGET opencv::opencv)
    set(ORBSLAM3_OPENCV_LINK opencv::opencv)
elseif(TARGET OpenCV::OpenCV)
    set(ORBSLAM3_OPENCV_LINK OpenCV::OpenCV)
elseif(TARGET OpenCV::opencv)
    set(ORBSLAM3_OPENCV_LINK OpenCV::opencv)
elseif(OpenCV_LIBS)
    set(ORBSLAM3_OPENCV_LINK ${OpenCV_LIBS})
else()
    message(FATAL_ERROR "OpenCV was found, but no usable CMake target or library list is available.")
endif()

set(ORBSLAM3_EXTRA_INCLUDE_DIRS)
if(OpenCV_INCLUDE_DIRS)
    list(APPEND ORBSLAM3_EXTRA_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
elseif(OpenCV_INCLUDE_DIR)
    list(APPEND ORBSLAM3_EXTRA_INCLUDE_DIRS ${OpenCV_INCLUDE_DIR})
endif()

if(TARGET Eigen3::Eigen)
    set(ORBSLAM3_EIGEN_LINK Eigen3::Eigen)
    get_target_property(_ORBSLAM3_EIGEN_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
    if(_ORBSLAM3_EIGEN_INCLUDE_DIRS)
        list(APPEND ORBSLAM3_EXTRA_INCLUDE_DIRS ${_ORBSLAM3_EIGEN_INCLUDE_DIRS})
    endif()
elseif(EIGEN3_INCLUDE_DIRS)
    list(APPEND ORBSLAM3_EXTRA_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
elseif(Eigen3_INCLUDE_DIRS)
    list(APPEND ORBSLAM3_EXTRA_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})
elseif(EIGEN3_INCLUDE_DIR)
    list(APPEND ORBSLAM3_EXTRA_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

set(ORB_SLAM3_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ORB_SLAM3)
set(ORB_G2O_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/orb_g2o)
file(MAKE_DIRECTORY ${ORB_G2O_GEN_DIR}/g2o/core)
file(MAKE_DIRECTORY ${ORB_G2O_GEN_DIR}/g2o/types)
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/g2o_config.h.in
    ${ORB_G2O_GEN_DIR}/config.h
    @ONLY
)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Observations.msg"
    "action/ExportPointCloud.action"
    "srv/GetTrackedPoints.srv"
    "srv/GetStatus.srv"
    "srv/LoadMap.srv"
    "srv/SaveMap.srv"
    "srv/SaveTrajectory.srv"
    "srv/SetLocalizationMode.srv"
    DEPENDENCIES builtin_interfaces sensor_msgs std_msgs
)

add_library(ORB_SLAM3 SHARED
    ${ORB_SLAM3_SRC_DIR}/src/System.cc
    ${ORB_SLAM3_SRC_DIR}/src/Tracking.cc
    ${ORB_SLAM3_SRC_DIR}/src/LocalMapping.cc
    ${ORB_SLAM3_SRC_DIR}/src/LoopClosing.cc
    ${ORB_SLAM3_SRC_DIR}/src/ORBextractor.cc
    ${ORB_SLAM3_SRC_DIR}/src/ORBmatcher.cc
    ${ORB_SLAM3_SRC_DIR}/src/FrameDrawer.cc
    ${ORB_SLAM3_SRC_DIR}/src/Converter.cc
    ${ORB_SLAM3_SRC_DIR}/src/MapPoint.cc
    ${ORB_SLAM3_SRC_DIR}/src/KeyFrame.cc
    ${ORB_SLAM3_SRC_DIR}/src/Atlas.cc
    ${ORB_SLAM3_SRC_DIR}/src/Map.cc
    ${ORB_SLAM3_SRC_DIR}/src/MapDrawer.cc
    ${ORB_SLAM3_SRC_DIR}/src/Optimizer.cc
    ${ORB_SLAM3_SRC_DIR}/src/Frame.cc
    ${ORB_SLAM3_SRC_DIR}/src/KeyFrameDatabase.cc
    ${ORB_SLAM3_SRC_DIR}/src/Sim3Solver.cc
    ${ORB_SLAM3_SRC_DIR}/src/Viewer.cc
    ${ORB_SLAM3_SRC_DIR}/src/ImuTypes.cc
    ${ORB_SLAM3_SRC_DIR}/src/G2oTypes.cc
    ${ORB_SLAM3_SRC_DIR}/src/CameraModels/Pinhole.cpp
    ${ORB_SLAM3_SRC_DIR}/src/CameraModels/KannalaBrandt8.cpp
    ${ORB_SLAM3_SRC_DIR}/src/OptimizableTypes.cpp
    ${ORB_SLAM3_SRC_DIR}/src/MLPnPsolver.cpp
    ${ORB_SLAM3_SRC_DIR}/src/GeometricTools.cc
    ${ORB_SLAM3_SRC_DIR}/src/TwoViewReconstruction.cc
    ${ORB_SLAM3_SRC_DIR}/src/Config.cc
    ${ORB_SLAM3_SRC_DIR}/src/Settings.cc
)
target_include_directories(ORB_SLAM3
    PUBLIC
    ${ORB_SLAM3_SRC_DIR}
    ${ORB_SLAM3_SRC_DIR}/include
    ${ORB_SLAM3_SRC_DIR}/include/CameraModels
    ${ORB_SLAM3_SRC_DIR}/Thirdparty/Sophus
    ${ORB_G2O_GEN_DIR}/g2o/core
    ${ORB_G2O_GEN_DIR}/g2o/types
    ${ORBSLAM3_EXTRA_INCLUDE_DIRS}
)
set_target_properties(ORB_SLAM3 PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(ORB_SLAM3 PRIVATE
        -Wno-deprecated-declarations
        -Wno-unused-parameter
        -Wno-reorder
        -Wno-sign-compare
        -Wno-vla
    )
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(ORB_SLAM3 PRIVATE
        -Wno-maybe-uninitialized
    )
endif()
target_compile_definitions(ORB_SLAM3 PRIVATE HAVE_GLEW GL_GLEXT_PROTOTYPES)
target_link_libraries(ORB_SLAM3
    PUBLIC
    ${ORBSLAM3_OPENCV_LINK}
    ${ORBSLAM3_EIGEN_LINK}
    Pangolin::pangolin
    OpenGL::GL
    GLEW::GLEW
    Sophus::Sophus
    g2o::g2o
    DBoW2::DBoW2
    -lboost_serialization
    -lcrypto
)

add_executable(orbslam3_core_node
    src/core/core_config.cpp
    src/core/command_completion.cpp
    src/core/guarded_worker.cpp
    src/core/resource_resolver.cpp
    src/core/tracking_diagnostics.cpp
    src/core/core_node_interfaces.cpp
    src/rgbd/rgbd_config.cpp
    src/rgbd/rgbd_image_convert.cpp
    src/core/tracking_backend.cpp
    src/core/core_node_config.cpp
    src/core/core_node_worker.cpp
    src/core/core_node_export.cpp
    src/core/core_node_main.cpp
)
add_executable(rgbd_frontend_node
    src/rgbd/rgbd_config.cpp
    src/rgbd/rgbd_image_convert.cpp
    src/rgbd/rgbd_frontend_node.cpp
)
add_executable(path_node
    src/tools/path_node.cpp
)
add_executable(tracking_cloud_node
    src/tools/tracking_cloud_node.cpp
)
add_executable(orbslam3_make_e2e_bag
    src/tools/make_e2e_bag.cpp
)
add_executable(orbslam3_native_runner
    src/tools/native_runner.cpp
)

set(ORBSLAM3_DIAG_COMPILE_DEFS)
set(ORBSLAM3_DIAG_COMPILE_OPTS)
if(ORBSLAM3_EIGEN_SAFE_MODE)
    list(APPEND ORBSLAM3_DIAG_COMPILE_DEFS
        EIGEN_DONT_VECTORIZE
        EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
        EIGEN_MAX_ALIGN_BYTES=0
    )
endif()
if(ORBSLAM3_DISABLE_FAST_MATH)
    list(APPEND ORBSLAM3_DIAG_COMPILE_OPTS -fno-fast-math)
endif()
if(ORBSLAM3_DIAG_COMPILE_DEFS)
    target_compile_definitions(ORB_SLAM3 PRIVATE ${ORBSLAM3_DIAG_COMPILE_DEFS})
    target_compile_definitions(orbslam3_core_node PRIVATE ${ORBSLAM3_DIAG_COMPILE_DEFS})
    target_compile_definitions(orbslam3_native_runner PRIVATE ${ORBSLAM3_DIAG_COMPILE_DEFS})
endif()
if(ORBSLAM3_DIAG_COMPILE_OPTS)
    target_compile_options(ORB_SLAM3 PRIVATE ${ORBSLAM3_DIAG_COMPILE_OPTS})
    target_compile_options(orbslam3_core_node PRIVATE ${ORBSLAM3_DIAG_COMPILE_OPTS})
    target_compile_options(orbslam3_native_runner PRIVATE ${ORBSLAM3_DIAG_COMPILE_OPTS})
endif()
if(ORBSLAM3_EIGEN_SAFE_MODE OR ORBSLAM3_DISABLE_FAST_MATH)
    message(STATUS "ORB_SLAM3 diagnostics: EIGEN_SAFE_MODE=${ORBSLAM3_EIGEN_SAFE_MODE}, DISABLE_FAST_MATH=${ORBSLAM3_DISABLE_FAST_MATH}")
endif()
target_include_directories(orbslam3_core_node
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${ORB_SLAM3_SRC_DIR}/include
    ${ORB_SLAM3_SRC_DIR}
)
target_include_directories(rgbd_frontend_node
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${ORB_SLAM3_SRC_DIR}/include
    ${ORB_SLAM3_SRC_DIR}
)
target_include_directories(path_node
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(tracking_cloud_node
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(orbslam3_make_e2e_bag
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(orbslam3_native_runner
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${ORB_SLAM3_SRC_DIR}/include
    ${ORB_SLAM3_SRC_DIR}
)
target_compile_features(orbslam3_core_node PUBLIC cxx_std_17)
target_compile_features(rgbd_frontend_node PUBLIC cxx_std_17)
target_compile_features(path_node PUBLIC cxx_std_17)
target_compile_features(tracking_cloud_node PUBLIC cxx_std_17)
target_compile_features(orbslam3_make_e2e_bag PUBLIC cxx_std_17)
target_compile_features(orbslam3_native_runner PUBLIC cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(orbslam3_core_node PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
    target_compile_options(rgbd_frontend_node PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
    target_compile_options(path_node PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
    target_compile_options(tracking_cloud_node PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
    target_compile_options(orbslam3_make_e2e_bag PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
    target_compile_options(orbslam3_native_runner PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
endif()
ament_target_dependencies(orbslam3_core_node
    ament_index_cpp
    diagnostic_updater
    diagnostic_msgs
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    lifecycle_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    std_srvs
    tf2_ros
    cv_bridge
)
ament_target_dependencies(rgbd_frontend_node
    ament_index_cpp
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
    std_msgs
    cv_bridge
)
ament_target_dependencies(path_node
    rclcpp
    geometry_msgs
    nav_msgs
    std_srvs
)
ament_target_dependencies(tracking_cloud_node
    rclcpp
    sensor_msgs
)
ament_target_dependencies(orbslam3_make_e2e_bag
    rclcpp
    rosbag2_cpp
    rosbag2_storage
    cv_bridge
    sensor_msgs
    std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(orbslam3_core_node "${cpp_typesupport_target}")
target_link_libraries(rgbd_frontend_node "${cpp_typesupport_target}")
target_link_libraries(path_node "${cpp_typesupport_target}")
target_link_libraries(tracking_cloud_node "${cpp_typesupport_target}")
target_link_libraries(orbslam3_make_e2e_bag "${cpp_typesupport_target}")
target_link_libraries(orbslam3_core_node
    ${ORBSLAM3_OPENCV_LINK}
    ORB_SLAM3
)
target_link_libraries(orbslam3_native_runner
    ORB_SLAM3
)
target_link_options(orbslam3_native_runner PRIVATE -Wl,--no-as-needed)
find_library(ORBSLAM3_SYSTEM_JPEG
    NAMES libjpeg.so jpeg
    PATHS /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu
    NO_DEFAULT_PATH
)
if(ORBSLAM3_SYSTEM_JPEG)
    target_link_libraries(orbslam3_native_runner ${ORBSLAM3_SYSTEM_JPEG})
endif()
target_compile_definitions(orbslam3_core_node PRIVATE HAVE_GLEW GL_GLEXT_PROTOTYPES)
target_compile_definitions(orbslam3_native_runner PRIVATE HAVE_GLEW GL_GLEXT_PROTOTYPES)

install(TARGETS
    orbslam3_core_node
    rgbd_frontend_node
    path_node
    tracking_cloud_node
    orbslam3_make_e2e_bag
    orbslam3_native_runner
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
    include/
    DESTINATION include
)

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)

    add_executable(obs_pub_node
        test/nodes/obs_pub_node.cpp
    )
    add_executable(svc_storm_node
        test/nodes/svc_storm_node.cpp
    )

    target_include_directories(obs_pub_node
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    target_include_directories(svc_storm_node
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    target_compile_features(obs_pub_node PUBLIC cxx_std_17)
    target_compile_features(svc_storm_node PUBLIC cxx_std_17)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(obs_pub_node PRIVATE
            -Wall
            -Wextra
            -Wpedantic
        )
        target_compile_options(svc_storm_node PRIVATE
            -Wall
            -Wextra
            -Wpedantic
        )
    endif()

    ament_target_dependencies(obs_pub_node
        rclcpp
        sensor_msgs
        std_msgs
    )
    ament_target_dependencies(svc_storm_node
        rclcpp
        rclcpp_action
        std_srvs
        sensor_msgs
        geometry_msgs
        nav_msgs
    )

    target_link_libraries(obs_pub_node "${cpp_typesupport_target}")
    target_link_libraries(svc_storm_node "${cpp_typesupport_target}")

    install(TARGETS
        obs_pub_node
        svc_storm_node
        DESTINATION lib/${PROJECT_NAME}
    )

    add_launch_test(test/launch/test_rgbd_smoke.launch.py TIMEOUT 90)
    add_launch_test(test/launch/test_thread_safety_storm.launch.py TIMEOUT 150)
    add_launch_test(test/launch/test_backpressure.launch.py TIMEOUT 150)
    add_launch_test(test/launch/test_export_under_load.launch.py TIMEOUT 150)
    add_launch_test(test/launch/test_long_run.launch.py TIMEOUT 220)
    add_launch_test(test/launch/test_e2e_lifecycle_bag.launch.py TIMEOUT 300)
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/ros_logs")
    set(_ORB_LAUNCH_TESTS
        test_launch_test_rgbd_smoke.launch.py
        test_launch_test_thread_safety_storm.launch.py
        test_launch_test_backpressure.launch.py
        test_launch_test_export_under_load.launch.py
        test_launch_test_long_run.launch.py
        test_launch_test_e2e_lifecycle_bag.launch.py
    )
    foreach(_test_name IN LISTS _ORB_LAUNCH_TESTS)
        set_tests_properties(${_test_name} PROPERTIES
            ENVIRONMENT "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
        )
    endforeach()

    ament_add_pytest_test(test_metrics
        test/pytest/test_metrics.py
        TIMEOUT 30
    )
    ament_add_pytest_test(test_services
        test/pytest/test_services.py
        TIMEOUT 30
    )
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
