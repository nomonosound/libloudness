prefix=@CMAKE_INSTALL_PREFIX@
includedir=${prefix}/include
libdir=${prefix}/@CMAKE_INSTALL_LIBDIR@

Name: libloudness
Description: ITU BS.1770 / EBU-R128 based loudness measurments
Version: @LOUDNESS_VERSION@
URL: https://github.com/nomonosound/libloudness
Libs: -L${libdir} -lloudness
Cflags: -I${includedir}
