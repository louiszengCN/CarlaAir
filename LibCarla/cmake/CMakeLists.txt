cmake_minimum_required(VERSION 3.20)
project(libcarla)

# Modern policies — avoids deprecation warnings on CMake 3.20+
cmake_policy(SET CMP0091 NEW) # MSVC runtime library via MSVC_RUNTIME_LIBRARY
cmake_policy(SET CMP0077 NEW) # option() honours variables already set by parent
cmake_policy(SET CMP0063 NEW) # visibility properties honoured for all targets

option(LIBCARLA_BUILD_DEBUG "Build debug configuration" ON)
option(LIBCARLA_BUILD_RELEASE "Build release configuration" ON)
option(LIBCARLA_BUILD_TEST "Build unit tests" ON)

message(STATUS "Build debug:   ${LIBCARLA_BUILD_DEBUG}")
message(STATUS "Build release: ${LIBCARLA_BUILD_RELEASE}")
message(STATUS "Build test:    ${LIBCARLA_BUILD_TEST}")

set(libcarla_source_path "${PROJECT_SOURCE_DIR}/../source")
set(libcarla_source_thirdparty_path "${libcarla_source_path}/third-party")

include_directories(${libcarla_source_path})
include_directories(${libcarla_source_thirdparty_path})

if (CARLA_VERSION)
  configure_file(${libcarla_source_path}/carla/Version.h.in ${libcarla_source_path}/carla/Version.h)
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Client")
  add_subdirectory("client")
elseif (CMAKE_BUILD_TYPE STREQUAL "Server")
  add_subdirectory("server")
elseif (CMAKE_BUILD_TYPE STREQUAL "Pytorch")
add_subdirectory("pytorch")
elseif (CMAKE_BUILD_TYPE STREQUAL "ros2")
add_subdirectory("fast_dds")
else ()
  message(FATAL_ERROR "Unknown build type '${CMAKE_BUILD_TYPE}'")
endif ()

# GTest is not compiled on Windows.
if ((LIBCARLA_BUILD_TEST) AND (NOT WIN32) AND (NOT (CMAKE_BUILD_TYPE STREQUAL "Pytorch")) AND (NOT (CMAKE_BUILD_TYPE STREQUAL "ros2")))
  add_subdirectory("test")
endif()
