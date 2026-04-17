cmake_minimum_required(VERSION 3.20)
project(CARLA)

# Modern policies — avoids deprecation warnings on CMake 3.20+
cmake_policy(SET CMP0091 NEW) # MSVC runtime library via MSVC_RUNTIME_LIBRARY
cmake_policy(SET CMP0077 NEW) # option() honours variables already set by parent
cmake_policy(SET CMP0063 NEW) # visibility properties honoured for all targets

include("Build/CMakeLists.txt.in")

add_subdirectory("LibCarla/cmake")
