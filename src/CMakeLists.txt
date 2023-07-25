option(LOUDNESS_WITH_STATIC_PIC "Compile static library with -fPIC flag" OFF)

if(MSVC AND (CMAKE_SIZEOF_VOID_P LESS 8))
  add_compile_options(/arch:SSE2)
endif()

set(LOUDNESS_VERSION_MAJOR 0)
set(LOUDNESS_VERSION 0.1.0)

add_library(loudness
  loudness/bs1770-calculator.cpp
  loudness/bs1770-calculator.hpp
  loudness/constants.hpp
  loudness/defines.hpp
  loudness/detail/meter-impl.cpp
  loudness/detail/meter-impl.hpp
  loudness/interpolator.cpp
  loudness/interpolator.hpp
  loudness/k-filter.cpp
  loudness/k-filter.hpp
  loudness/meter.hpp
  loudness/utils.hpp
)
target_include_directories(loudness PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(loudness PUBLIC gcem PRIVATE BS_thread_pool)

set_target_properties(loudness PROPERTIES
  SOVERSION ${LOUDNESS_VERSION_MAJOR}
  VERSION ${LOUDNESS_VERSION}
)

if(NOT BUILD_SHARED_LIBS)
  if(LOUDNESS_WITH_STATIC_PIC)
    set_property(TARGET loudness PROPERTY POSITION_INDEPENDENT_CODE ON)
  endif()
endif()

install(FILES loudness/meter.hpp loudness/defines.hpp loudness/detail/meter-impl.hpp
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/loudness)
install(TARGETS loudness
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

### pkg-config
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/libloudness.pc.cmake
               ${CMAKE_CURRENT_BINARY_DIR}/libloudness.pc @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/libloudness.pc"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
