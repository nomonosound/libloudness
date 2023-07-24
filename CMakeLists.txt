cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
project(libloudness)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_SHARED_LIBS "Build shared libraries instead of static ones" ON)

include(GNUInstallDirs)

include_directories(src)

add_subdirectory(vendor)
add_subdirectory(src)
add_subdirectory(test)


if(BUILD_SHARED_LIBS)
  message(STATUS "Building shared library (set BUILD_SHARED_LIBS to OFF to build static)")
else()
  message(STATUS "Building static library")
endif()

if(ENABLE_TESTS)
  message(STATUS "building tests!")
else()
  message(STATUS "not building tests, set ENABLE_TESTS to ON to enable")
endif()
