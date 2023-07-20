set(BUILD_STATIC_LIBS       ON  CACHE BOOL "Build static library")
set(WITH_STATIC_PIC         OFF CACHE BOOL "Compile static library with -fPIC flag")


if(MSVC)
  add_definitions(-D_USE_MATH_DEFINES)
  if(CMAKE_SIZEOF_VOID_P LESS 8)
    add_definitions(/arch:SSE2)
  endif()
endif()


set(LOUDNESS_VERSION_MAJOR 0)
set(LOUDNESS_VERSION 0.1.0)

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

target_include_directories(loudness PUBLIC ${CMAKE_SOURCE_DIR}/vendor/thread-pool/include)
target_link_libraries(loudness gcem)

set_target_properties(loudness PROPERTIES
  SOVERSION ${LOUDNESS_VERSION_MAJOR}
  VERSION ${LOUDNESS_VERSION}
)

if(BUILD_SHARED_LIBS)
else()
  if(WITH_STATIC_PIC)
    set_property(TARGET loudness PROPERTY POSITION_INDEPENDENT_CODE ON)
  endif()
endif()

install(FILES meter.hpp detail/meter-impl.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS loudness
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

### pkg-config
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/libloudness.pc.cmake
               ${CMAKE_CURRENT_BINARY_DIR}/libloudness.pc @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/libloudness.pc"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
