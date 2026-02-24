# cmake/orbslam3_vendor.cmake
include(ExternalProject)

# use build script instead
get_filename_component(_PKG_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(_SCRIPT "${_PKG_ROOT}/third_party/build_orbslam3.py")

ExternalProject_Add(orbslam3_ep
  DEPENDS pangolin_ep
  SOURCE_DIR "${_PKG_ROOT}/third_party/ORB_SLAM3"
  BINARY_DIR "${CMAKE_BINARY_DIR}/_deps/orbslam3-dummy"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND
  ${CMAKE_COMMAND} -E env
  ORB_BUILD_JOBS=8
  ORB_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  ORB_CMAKE_GENERATOR=Ninja
  ORB_BUILD_TYPE=Release
  python3 ${_SCRIPT}
  INSTALL_COMMAND ""
  UPDATE_COMMAND ""
)
