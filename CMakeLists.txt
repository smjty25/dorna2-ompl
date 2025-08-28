cmake_minimum_required(VERSION 3.20)
project(ompl_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Policies
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 NEW)
endif()

# Auto-use vcpkg toolchain if available
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{VCPKG_ROOT})
  set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE FILEPATH "")
endif()

# --- Python / pybind11 ---
# Pass your interpreter at configure time:
#   -D Python3_EXECUTABLE="C:/Path/To/Python312/python.exe"
set(PYBIND11_FINDPYTHON ON)
set(Python3_FIND_VIRTUALENV FIRST)
set(Python3_FIND_STRATEGY LOCATION)
set(Python3_FIND_REGISTRY LAST)

find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# Compute site-packages of THIS interpreter (portable & robust)
execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c "import sysconfig,sys; sys.stdout.write(sysconfig.get_path('platlib'))"
  OUTPUT_VARIABLE PY_SITEARCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
# Normalize slashes for Windows so install scripts don't see '\x' escapes
if(COMMAND cmake_path)
  cmake_path(CONVERT "${PY_SITEARCH}" TO_CMAKE_PATH_LIST PY_SITEARCH_CMAKE)
else()
  file(TO_CMAKE_PATH "${PY_SITEARCH}" PY_SITEARCH_CMAKE)
endif()

# --- Dependencies (via vcpkg) ---
find_package(ompl CONFIG REQUIRED)
find_package(fcl CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(urdfdom_headers CONFIG REQUIRED)
find_package(urdfdom CONFIG REQUIRED)

# --- Resources (URDF etc.) ---
include(GNUInstallDirs)
set(RESOURCE_SOURCE_DIR "${CMAKE_SOURCE_DIR}/resources")
set(OMPL_DEMO_BUILD_RESOURCE_DIR   "${CMAKE_BINARY_DIR}/resources")
set(OMPL_DEMO_INSTALL_RESOURCE_DIR "${CMAKE_INSTALL_FULL_DATADIR}/ompl_demo")

if(EXISTS "${RESOURCE_SOURCE_DIR}")
  add_custom_target(copy_resources ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${RESOURCE_SOURCE_DIR}"
            "${OMPL_DEMO_BUILD_RESOURCE_DIR}"
    COMMENT "Copying resources to build directory")
endif()

configure_file(
  "${CMAKE_SOURCE_DIR}/cmake/config.hpp.in"
  "${CMAKE_BINARY_DIR}/generated/config.hpp"
  @ONLY
)

# --- Python extension module ---
pybind11_add_module(dornaompl
  src/bindings.cpp
  # src/your_other_sources.cpp
)
target_link_libraries(dornaompl PRIVATE
  ompl::ompl
  fcl
  Eigen3::Eigen
  urdfdom_headers::urdfdom_headers
  urdfdom::urdfdom_model
)
target_include_directories(dornaompl PRIVATE "${CMAKE_BINARY_DIR}/generated")
if(TARGET copy_resources)
  add_dependencies(dornaompl copy_resources)
endif()
set_target_properties(dornaompl PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

# --- Install: module to site-packages; resources next to it ---
install(TARGETS dornaompl
  LIBRARY DESTINATION "${PY_SITEARCH_CMAKE}"
  RUNTIME DESTINATION "${PY_SITEARCH_CMAKE}"   # .pyd on Windows
)
if(EXISTS "${RESOURCE_SOURCE_DIR}")
  install(DIRECTORY "${RESOURCE_SOURCE_DIR}/"
          DESTINATION "${PY_SITEARCH_CMAKE}/dornaompl_data")
endif()
