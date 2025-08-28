cmake_minimum_required(VERSION 3.21) # TARGET_RUNTIME_DLLS needs >= 3.21
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

# --- Python selection (NO find_package to avoid vcpkg wrappers) ---
# Expect: -D Python3_EXECUTABLE="C:/Users/you/AppData/Local/Programs/Python/Python39/python.exe"
if(NOT DEFINED Python3_EXECUTABLE)
  message(FATAL_ERROR "Pass your interpreter with -D Python3_EXECUTABLE=... (python.exe)")
endif()

# Pull major+minor (e.g., 39), sys.exec_prefix, and site-packages (platlib)
execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c "import sys,sysconfig; mm=f'{sys.version_info[0]}{sys.version_info[1]}'; print(mm); print(sys.exec_prefix); print(sysconfig.get_path('platlib'))"
  OUTPUT_VARIABLE _PY_INFO
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
string(REPLACE "\r\n" "\n" _PY_INFO "${_PY_INFO}")
string(REPLACE "\r"   "\n" _PY_INFO "${_PY_INFO}")
string(REPLACE "\n" ";" _PY_INFO "${_PY_INFO}")
list(GET _PY_INFO 0 PY_MM)
list(GET _PY_INFO 1 PY_ROOT)
list(GET _PY_INFO 2 PY_SITEARCH)

set(PY_INCLUDE "${PY_ROOT}/include")
set(PY_LIB     "${PY_ROOT}/libs/python${PY_MM}.lib")

if(NOT EXISTS "${PY_INCLUDE}/Python.h")
  message(FATAL_ERROR "Python headers not found at: ${PY_INCLUDE}/Python.h")
endif()
if(NOT EXISTS "${PY_LIB}")
  message(FATAL_ERROR "Python import lib not found: ${PY_LIB}")
endif()

# Normalize slashes so install() paths are clean on Windows
if(COMMAND cmake_path)
  cmake_path(CONVERT "${PY_SITEARCH}" TO_CMAKE_PATH_LIST PY_SITEARCH_CMAKE)
else()
  file(TO_CMAKE_PATH "${PY_SITEARCH}" PY_SITEARCH_CMAKE)
endif()

# --- pybind11 (don’t let it search Python; we provide it) ---
# Tell pybind11 NOT to call find_package(Python...) internally.
set(PYBIND11_FINDPYTHON OFF)
# Provide legacy vars some pybind11 configs consult.
set(PYTHON_EXECUTABLE  "${Python3_EXECUTABLE}")
set(PYTHON_INCLUDE_DIR "${PY_INCLUDE}")
set(PYTHON_LIBRARY     "${PY_LIB}")

find_package(pybind11 CONFIG REQUIRED)

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
  "${PY_LIB}"                       # link your chosen Python explicitly
)

target_include_directories(dornaompl PRIVATE
  "${CMAKE_BINARY_DIR}/generated"
  "${PY_INCLUDE}"                   # include your chosen Python headers
)

if(TARGET copy_resources)
  add_dependencies(dornaompl copy_resources)
endif()

set_target_properties(dornaompl PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

# --- Copy dependent DLLs next to the .pyd (Option A) ---
if(MSVC)
  add_custom_command(TARGET dornaompl POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "Copying runtime DLLs for dornaompl..."
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:dornaompl>
            $<TARGET_FILE_DIR:dornaompl>
    COMMAND_EXPAND_LISTS
  )
endif()

# --- Install: module to site-packages; resources next to it ---
install(TARGETS dornaompl
  LIBRARY DESTINATION "${PY_SITEARCH_CMAKE}"
  RUNTIME DESTINATION "${PY_SITEARCH_CMAKE}"   # .pyd on Windows
  ARCHIVE DESTINATION "${PY_SITEARCH_CMAKE}"
)

# Copy all dependent DLLs beside the installed .pyd
install(FILES $<TARGET_RUNTIME_DLLS:dornaompl>
        DESTINATION "${PY_SITEARCH_CMAKE}"
        CONFIGURATIONS Release RelWithDebInfo MinSizeRel)

if(EXISTS "${RESOURCE_SOURCE_DIR}")
  install(DIRECTORY "${RESOURCE_SOURCE_DIR}/"
          DESTINATION "${PY_SITEARCH_CMAKE}/dornaompl_data")
endif()

