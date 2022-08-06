# TurboJpeg, some study using turbojpeg

find_package(TurboJpeg QUIET)     # turbojpeg

if(TurboJpeg_FOUND)
    list(APPEND DEPEND_INCLUDES ${TurboJpeg_INCLUDE_DIRS})
    list(APPEND DEPEND_LIBS ${TurboJpeg_LIBRARIES})

    add_subdirectory(TurboJpeg01_CompressBenchmark)
endif()