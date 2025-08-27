cmake_minimum_required(VERSION 3.20)
project(ompl_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

# ---- Dependencies (via vcpkg manifest) ----
find_package(ompl CONFIG REQUIRED)
find_package(fcl CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(urdfdom_headers CONFIG REQUIRED)   # headers-only
find_package(urdfdom CONFIG REQUIRED)

# ---- Executable ----
add_executable(ompl_demo src/main.cpp)
target_link_libraries(ompl_demo PRIVATE
  ompl::ompl
  fcl
  Eigen3::Eigen
  urdfdom_headers::urdfdom_headers
  urdfdom::urdfdom_model
)

# ---- Resources (URDF etc.) ----
# Place your URDF at: ${CMAKE_SOURCE_DIR}/resources/urdf/DornaTA_500mm_hd_rail.urdf
include(GNUInstallDirs)

set(RESOURCE_SOURCE_DIR "${CMAKE_SOURCE_DIR}/resources")
set(OMPL_DEMO_BUILD_RESOURCE_DIR   "${CMAKE_BINARY_DIR}/resources")
set(OMPL_DEMO_INSTALL_RESOURCE_DIR "${CMAKE_INSTALL_FULL_DATADIR}/ompl_demo")

# Copy resources into the build dir so running from the build tree works
if(EXISTS "${RESOURCE_SOURCE_DIR}")
  add_custom_target(copy_resources ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${RESOURCE_SOURCE_DIR}"
            "${OMPL_DEMO_BUILD_RESOURCE_DIR}"
    COMMENT "Copying resources to build directory"
  )
  add_dependencies(ompl_demo copy_resources)
endif()

# Generate a small config header with the resource paths
configure_file(
  "${CMAKE_SOURCE_DIR}/cmake/config.hpp.in"
  "${CMAKE_BINARY_DIR}/generated/config.hpp"
  @ONLY
)
target_include_directories(ompl_demo PRIVATE "${CMAKE_BINARY_DIR}/generated")

# ---- Install (binary + resources) ----
install(TARGETS ompl_demo RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
if(EXISTS "${RESOURCE_SOURCE_DIR}")
  install(DIRECTORY "${RESOURCE_SOURCE_DIR}/" DESTINATION "${CMAKE_INSTALL_DATADIR}/ompl_demo")
endif()
