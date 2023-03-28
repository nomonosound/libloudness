cmake_minimum_required(VERSION 2.8.12)

set(ENABLE_TESTS OFF CACHE BOOL "Build test binaries, needs libsndfile")
set(ENABLE_FUZZER OFF CACHE BOOL "Build fuzzer binary")


include(FetchContent)

FetchContent_Declare(doctest
                     GIT_REPOSITORY https://github.com/doctest/doctest.git
                     GIT_TAG        v2.4.10) # update version number if needed
FetchContent_MakeAvailable(doctest)


add_executable(loudness_tests EXCLUDE_FROM_ALL)
target_sources(loudness_tests
    PRIVATE
    main.cpp
    test-k-filter.cpp
    test-ebur128.cpp
    test-bs1770-calculator.cpp
    test-utilities.hpp
)
target_link_libraries(loudness_tests PRIVATE doctest::doctest lufs_meter)



if(ENABLE_FUZZER)
  include_directories(${EBUR128_INCLUDE_DIR})

  add_executable(fuzzer fuzzer)
  target_link_libraries(fuzzer ebur128)
  target_compile_options(fuzzer PUBLIC "${FUZZER_FLAGS}")
  target_link_libraries(fuzzer "${FUZZER_FLAGS}")
endif()
