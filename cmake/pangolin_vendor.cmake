# cmake/pangolin_vendor.cmake
include(ExternalProject)

set(PANGOLIN_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/../third_party/Pangolin")
set(PANGOLIN_BUILD_DIR "${CMAKE_BINARY_DIR}/_deps/pangolin-build")

ExternalProject_Add(pangolin_ep
  SOURCE_DIR "${PANGOLIN_SRC_DIR}"
  BINARY_DIR "${PANGOLIN_BUILD_DIR}"
  CMAKE_ARGS
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON
  -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  -DBUILD_PANGOLIN_PYTHON=OFF
  BUILD_COMMAND ${CMAKE_COMMAND} --build . --config Release -j
  INSTALL_COMMAND ${CMAKE_COMMAND} --build . --target install --config Release
  UPDATE_COMMAND ""
)

# expose where it will install
set(Pangolin_DIR "${CMAKE_INSTALL_PREFIX}/lib/cmake/Pangolin" CACHE PATH "" FORCE)
