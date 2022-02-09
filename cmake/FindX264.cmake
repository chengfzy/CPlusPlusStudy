# Find package module for libx264
#
# The following variables are set by this module:
#
#   - X264_FOUND:           TRUE if libx264 is found
#   - X264_INCLUDE_DIRS:    Include directories for libx264
#   - X264_LIBRARIES:       Libraries for libx264
#
# The following variables control the behavior of this module:
#
#   - X264_INCLUDE_DIR_HINTS:   List of additional directories in which to search for X264 includes
#   - X264_LIBRARY_DIR_HINTS:   List of additional directories in which to search for X264 libraries

set(X264_INCLUDE_DIR_HINTS "" CACHE PATH "X264 include directory")
set(X264_LIBRARY_DIR_HINTS "" CACHE PATH "X264 library directory")

unset(X264_FOUND)
unset(X264_INCLUDE_DIRS)
unset(X264_LIBRARIES)

include(FindPackageHandleStandardArgs)

# use pkg-config to get the directories and then use these values in the find_path() and find_library() calls
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(_x264 x264 QUIET)
endif()

# set check include dir and lib dir, also the check suffixes
list(APPEND X264_CHECK_INCLUDE_DIRS
    ${_x264_INCLUDE_DIRS}
    ${_x264_INCLUDEDIR}
    /usr/local/include
    /usr/local/homebrew/include
    /opt/local/var/macports/software
    /opt/local/include
    /usr/include
)
list(APPEND X264_CHECK_PATH_SUFFIXES)
list(APPEND X264_CHECK_LIBRARY_DIRS
    ${_x264_LIBRARY_DIRS}
    ${_x264_LIBDIR}
    /usr/local/lib
    /usr/local/homebrew/lib
    /opt/local/lib
    /usr/lib
)
list(APPEND X264_CHECK_LIBRARY_SUFFIXES)

# find x264 include
find_path(X264_INCLUDE_DIRS
    NAMES
    x264.h
    PATHS
    ${X264_INCLUDE_DIR_HINTS}
    ${X264_CHECK_INCLUDE_DIRS}
    PATH_SUFFIXES
    ${X264_CHECK_PATH_SUFFIXES}
)
# find x264 lib
find_library(X264_LIBRARIES
    NAMES
    x264
    libx264
    PATHS
    ${X264_LIBRARY_DIR_HINTS}
    ${X264_CHECK_LIBRARY_DIRS}
    PATH_SUFFIXES
    ${X264_CHECK_LIBRARY_SUFFIXES}
)

if (X264_INCLUDE_DIRS AND X264_LIBRARIES)
    set(X264_FOUND TRUE)
    
    # parse version
    set(_X264VersionHeader "${X264_INCLUDE_DIRS}/x264_config.h")
    if (EXISTS "${_X264VersionHeader}")
        file(STRINGS "${_X264VersionHeader}" _X264Version  REGEX "X264_POINTVER")
        string(REGEX REPLACE ".*\"n?\(.*\)\"" "\\1" X264Version "${_X264Version}")
    endif()

    # show message
    message(STATUS "Found X264 (Version: ${X264Version})")
    message(STATUS "  Includes: ${X264_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${X264_LIBRARIES}")
else()
    set(X264_FOUND FALSE)
    if (X264_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find X264")
    endif()
endif()

find_package_handle_standard_args(X264
    REQUIRED_VARS X264_INCLUDE_DIRS
    VERSION_VAR X264Version
)