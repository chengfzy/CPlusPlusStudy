# Find package module for TurboJpeg library.
#
# The following variables are set by this module:
#
#   TurboJpeg_FOUND: TRUE if TurboJpeg is found.
#   TurboJpeg_INCLUDE_DIRS: Include directories for TurboJpeg.
#   TurboJpeg_LIBRARIES: Libraries required to link TurboJpeg.
#
# The following variables control the behavior of this module:
#
# TurboJpeg_INCLUDE_DIR_HINTS: List of additional directories in which to search for TurboJpeg includes.
# TurboJpeg_LIBRARY_DIR_HINTS: List of additional directories in which to search for TurboJpeg libraries.

set(TurboJpeg_INCLUDE_DIR_HINTS "" CACHE PATH "TurboJpeg include directory")
set(TurboJpeg_LIBRARY_DIR_HINTS "" CACHE PATH "TurboJpeg library directory")

unset(TurboJpeg_FOUND)
unset(TurboJpeg_INCLUDE_DIRS)
unset(TurboJpeg_LIBRARIES)

include(FindPackageHandleStandardArgs)

list(APPEND TurboJpeg_CHECK_INCLUDE_DIRS
        /usr/local/include
        /usr/local/homebrew/include
        /opt/local/var/macports/software
        /opt/local/include
        /usr/include
)
list(APPEND TurboJpeg_CHECK_PATH_SUFFIXES)

list(APPEND TurboJpeg_CHECK_LIBRARY_DIRS
        /usr/local/lib
        /usr/local/homebrew/lib
        /opt/local/lib
        /usr/lib
)
list(APPEND TurboJpeg_CHECK_LIBRARY_SUFFIXES)

# find turbo-jpeg include
find_path(TurboJpeg_INCLUDE_DIRS
        NAMES
        turbojpeg.h
        PATHS
        ${TurboJpeg_INCLUDE_DIR_HINTS}
        ${TurboJpeg_CHECK_INCLUDE_DIRS}
        PATH_SUFFIXES
        ${TurboJpeg_CHECK_PATH_SUFFIXES}
)
# find turbo-jpeg lib
find_library(TurboJpeg_LIBRARIES
        NAMES
        turbojpeg
        libturbojpeg
        PATHS
        ${TurboJpeg_LIBRARY_DIR_HINTS}
        ${TurboJpeg_CHECK_LIBRARY_DIRS}
        PATH_SUFFIXES
        ${TurboJpeg_CHECK_LIBRARY_SUFFIXES}
)
# find jpeg lib
find_library(Jpeg_LIBRARIES
        NAMES
        jpeg
        libjpeg
        PATHS
        ${TurboJpeg_LIBRARY_DIR_HINTS}
        ${TurboJpeg_CHECK_LIBRARY_DIRS}
        PATH_SUFFIXES
        ${TurboJpeg_CHECK_LIBRARY_SUFFIXES}
)

if (TurboJpeg_INCLUDE_DIRS AND TurboJpeg_LIBRARIES)
    set(TurboJpeg_FOUND TRUE)
    set(TurboJpeg_LIBRARIES ${TurboJpeg_LIBRARIES} ${Jpeg_LIBRARIES})
    message(STATUS "Found TurboJpeg")
    message(STATUS "  Includes: ${TurboJpeg_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${TurboJpeg_LIBRARIES}")
else ()
    set(TurboJpeg_FOUND FALSE)
    if (TurboJpeg_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find TurboJpeg")
    endif ()
endif ()
