cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)
project(libloudness)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_SHARED_LIBS
    "Build shared libraries instead of static ones" ON)


list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR})

include(GNUInstallDirs)

include_directories(src)

add_subdirectory(vendor)
add_subdirectory(src)
add_subdirectory(test)

##### Print status
message(STATUS "Status          found / disabled --")

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
