set(BUILD_STATIC_LIBS       ON  CACHE BOOL "Build static library")
set(WITH_STATIC_PIC         OFF CACHE BOOL "Compile static library with -fPIC flag")


if(MSVC)
  add_definitions(-D_USE_MATH_DEFINES)
  if(CMAKE_SIZEOF_VOID_P LESS 8)
    add_definitions(/arch:SSE2)
  endif()
endif()


set(EBUR128_VERSION_MAJOR 1)
set(EBUR128_VERSION 1.2.6)

add_library(lufs_meter
    ebur128.cpp
    ebur128.hpp
    k-filter.cpp
    k-filter.hpp
    interpolator.cpp
    interpolator.hpp
    true-peak-calculator.cpp
    true-peak-calculator.hpp
    bs1770-calculator.cpp
    bs1770-calculator.hpp
    utils.hpp
    constants.hpp
)

target_link_libraries(lufs_meter gcem)

set_target_properties(lufs_meter PROPERTIES
  SOVERSION ${EBUR128_VERSION_MAJOR}
  VERSION ${EBUR128_VERSION}
)

if(BUILD_SHARED_LIBS)
else()
  if(WITH_STATIC_PIC)
    set_property(TARGET ebur128 PROPERTY POSITION_INDEPENDENT_CODE ON)
  endif()
endif()

# Link with Math library if available
find_library(MATH_LIBRARY m)
if(MATH_LIBRARY)
  target_link_libraries(lufs_meter ${MATH_LIBRARY})
endif()

if(ENABLE_FUZZER)
  target_compile_options(lufs_meter PUBLIC "${FUZZER_FLAGS}")
  target_compile_definitions(lufs_meter PRIVATE malloc=my_malloc calloc=my_calloc)
  target_link_libraries(lufs_meter "${SANITIZER_FLAGS}")
endif()

set(EBUR128_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "")

install(FILES ebur128.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS lufs_meter
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

#### pkg-config
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/libebur128.pc.cmake
               ${CMAKE_CURRENT_BINARY_DIR}/libebur128.pc @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/libebur128.pc"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
