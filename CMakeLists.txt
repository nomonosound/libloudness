cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
project(libloudness)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_SHARED_LIBS "Build shared libraries instead of static ones" ON)
option(LOUDNESS_ENABLE_TESTS "Build test binaries, needs libsndfile" OFF)

include(GNUInstallDirs)

add_subdirectory(vendor)
add_subdirectory(src)

if(LOUDNESS_ENABLE_TESTS)
  add_subdirectory(test)
endif()


if(BUILD_SHARED_LIBS)
  message(STATUS "Building shared library (set BUILD_SHARED_LIBS to OFF to build static)")
else()
  message(STATUS "Building static library")
endif()

if(LOUDNESS_ENABLE_TESTS)
  message(STATUS "building tests!")
else()
  message(STATUS "not building tests, set ENABLE_TESTS to ON to enable")
endif()
