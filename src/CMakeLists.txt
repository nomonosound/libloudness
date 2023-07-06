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

add_library(loudness
    meter.hpp
    defines.hpp
    detail/meter-impl.hpp
    detail/meter-impl.cpp
    k-filter.cpp
    k-filter.hpp
    interpolator.cpp
    interpolator.hpp
    bs1770-calculator.cpp
    bs1770-calculator.hpp
    utils.hpp
    constants.hpp
)

find_package(TBB REQUIRED)
target_link_libraries(loudness TBB::tbb gcem)

set_target_properties(loudness PROPERTIES
  SOVERSION ${EBUR128_VERSION_MAJOR}
  VERSION ${EBUR128_VERSION}
)

if(BUILD_SHARED_LIBS)
else()
  if(WITH_STATIC_PIC)
    set_property(TARGET loudness PROPERTY POSITION_INDEPENDENT_CODE ON)
  endif()
endif()

install(FILES meter.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS loudness
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

##### pkg-config
#configure_file(${CMAKE_CURRENT_SOURCE_DIR}/libebur128.pc.cmake
#               ${CMAKE_CURRENT_BINARY_DIR}/libebur128.pc @ONLY)
#install(FILES "${CMAKE_CURRENT_BINARY_DIR}/libebur128.pc"
#        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
