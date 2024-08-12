cmake_minimum_required(VERSION 3.14)

Include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.3.2
)

FetchContent_MakeAvailable(Catch2)

add_executable(loudness_tests EXCLUDE_FROM_ALL)
target_sources(loudness_tests
  PRIVATE
  test-ebur128.cpp
  test-meter.cpp
  test-utilities.hpp
)
target_link_libraries(loudness_tests PRIVATE Catch2::Catch2WithMain loudness)
target_link_libraries(loudness_tests PRIVATE sndfile)
target_compile_definitions(loudness_tests PRIVATE TEST_DIR="${CMAKE_SOURCE_DIR}/test/")
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(CTest)
include(Catch)
catch_discover_tests(loudness_tests)
