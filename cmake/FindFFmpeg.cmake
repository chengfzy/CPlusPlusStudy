# Find package module for FFmpeg
#
# The following variables are set by this module:
#
#   - FFmpeg_FOUND:         TRUE if FFmpeg is found
#   - FFmpeg_INCLUDE_DIRS:  Include directories for FFmpeg
#   - FFmpeg_LIBRARIES:     Libraries for FFmpeg
#
# The following variables control the behavior of this module:
#
#   - FFmpeg_INCLUDE_DIR_HINTS:     List of additional directories in which to search for FFmpeg includes
#   - FFmpeg_LIBRARY_DIR_HINTS:     List of additional directories in which to search for FFmpeg libraries
# 
#
# The following components are supported:
#   - avcodec
#   - avdevice
#   - avfilter
#   - avformat
#   - avresample
#   - avutil
#   - swresample
#   - swscale
# 
# For each component, the following are provided:
# 
#   - FFmpeg_<component>_FOUND:         TRUE if this compoment is found
#   - FFmpeg_<component>INCLUDE_DIRS:   Include directories for this component
#   - FFmpeg_<component>LIBRARIES:      Libraries for this component
#   - FFmpeg_<component>_LIBRARIES:     Libraries for the component.
#   - FFmpeg::<component>:              A target to use with target_link_libraries.

set(FFmpeg_INCLUDE_DIR_HINTS "" CACHE PATH "FFmpeg include directory")
set(FFmpeg_LIBRARY_DIR_HINTS "" CACHE PATH "FFmpeg library directory")
mark_as_advanced(
    FFmpeg_INCLUDE_DIR_HINTS
    FFmpeg_LIBRARY_DIR_HINTS
)

unset(FFmpeg_FOUND)
unset(FFmpeg_INCLUDE_DIRS)
unset(FFmpeg_LIBRARIES)

include(FindPackageHandleStandardArgs)

# function for find FFmpeg component
function(_FFmpegFind Component)
    # use pkg-config to get the directories and then use these values in the find_path() and find_library() calls
    find_package(PkgConfig QUIET)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(_FFmpeg_${Component} ${Component} QUIET)
    endif()

    # set check include dir and lib dir, also the check suffixes
    list(APPEND FFmpeg_${Component}_CHECK_INCLUDE_DIRS
        ${_FFmpeg_${Component}_INCLUDE_DIRS}
        ${_FFmpeg_${Component}_INCLUDEDIR}
        /usr/local/include
        /usr/local/homebrew/include
        /opt/local/var/macports/software
        /opt/local/include
        /usr/include
    )
    list(APPEND FFmpeg_${Component}_CHECK_PATH_SUFFIXES)
    list(APPEND FFmpeg_${Component}_CHECK_LIBRARY_DIRS
        ${_FFmpeg_${Component}_LIBRARY_DIRS}
        ${_FFmpeg_${Component}_LIBDIR}
        /usr/local/lib
        /usr/local/homebrew/lib
        /opt/local/lib
        /usr/lib
    )
    list(APPEND FFmpeg_${Component}_CHECK_LIBRARY_SUFFIXES)

    # find component include
    find_path(FFmpeg_${Component}_INCLUDE_DIRS
        NAMES
        "lib${Component}/${Component}.h"
        PATHS
        ${FFmpeg_INCLUDE_DIR_HINTS}
        ${FFmpeg_${Component}_CHECK_INCLUDE_DIRS}
        PATH_SUFFIXES
        ${FFmpeg_${Component}_CHECK_PATH_SUFFIXES}
    )
    # find FFmpeg lib
    find_library(FFmpeg_${Component}_LIBRARIES
        NAMES
        "${Component}"
        "lib${Component}"
        PATHS
        ${FFmpeg_LIBRARY_DIR_HINTS}
        ${FFmpeg_${Component}_CHECK_LIBRARY_DIRS}
        PATH_SUFFIXES
        ${FFmpeg_${Component}_CHECK_LIBRARY_SUFFIXES}
    )
    mark_as_advanced(
        FFmpeg_${Component}_INCLUDE_DIRS
        FFmpeg_${Component}_LIBRARIES
    )

    # set found status
    if(FFmpeg_${Component}_INCLUDE_DIRS AND FFmpeg_${Component}_LIBRARIES)
        set(FFmpeg_${Component}_FOUND TRUE PARENT_SCOPE)
        
        # parse version
        set(_VersionHeader "${FFmpeg_${Component}_INCLUDE_DIRS}/lib${Component}/version.h")
        if (EXISTS "${_VersionHeader}")
            string(TOUPPER "${Component}" Component_Upper)
            file(STRINGS "${_VersionHeader}" Version 
                REGEX "#define  *LIB${Component_Upper}_VERSION_(MAJOR|MINOR|MICRO)"
            )
            string(REGEX REPLACE ".*_MAJOR *\([0-9]*\).*" "\\1" MajorVersion "${Version}")
            string(REGEX REPLACE ".*_MINOR *\([0-9]*\).*" "\\1" MinorVersion "${Version}")
            string(REGEX REPLACE ".*_MICRO *\([0-9]*\).*" "\\1" MicroVersion "${Version}")
            if (NOT MajorVersion STREQUAL "" AND
                NOT MinorVersion STREQUAL "" AND
                NOT MicroVersion STREQUAL "")
                set(FFmpeg_${Component}_VERSION "${MajorVersion}.${MinorVersion}.${MicroVersion}")
            endif()
        endif()

        # set target
        if(NOT TARGET "FFmpeg::${Component}")
            add_library("FFmpeg::${Component}" UNKNOWN IMPORTED)
            set_target_properties("FFmpeg::${Component}" PROPERTIES
                IMPORTED_LOCATION "${FFmpeg_${Component}_LIBRARIES}"
                INTERFACE_INCLUDE_DIRECTORIES "${FFmpeg_${Component}_INCLUDE_DIRS}"
                INTERFACE_LINK_LIBRARIES "${FFmpeg_${Component}_LIBRARIES}"
            )
        endif()
        
        # show message
        message(STATUS "Found FFmpeg::${Component} (Version: ${FFmpeg_${Component}_VERSION})")
        # message(STATUS "  Includes: ${FFmpeg_${Component}_INCLUDE_DIRS}")
        # message(STATUS "  Libraries: ${FFmpeg_${Component}_LIBRARIES}")
    else()
        set(FFmpeg_${Component}_FOUND FALSE PARENT_SCOPE)
        if (FFmpeg_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find FFmpeg::${Component}")
        endif()
    endif()
endfunction()


# The default components were taken from a survey over other FindFFmpeg.cmake files
if(NOT FFmpeg_FIND_COMPONENTS)
  set(FFmpeg_FIND_COMPONENTS avcodec avformat avutil)
endif()

# find each component
set(FFmpeg_FOUND TRUE)
foreach(Component IN LISTS FFmpeg_FIND_COMPONENTS)
    _FFmpegFind(${Component})
    if(FFmpeg_${Component}_FOUND)
        list(APPEND FFmpeg_INCLUDE_DIRS ${FFmpeg_${Component}_INCLUDE_DIRS})
        list(APPEND FFmpeg_LIBRARIES ${FFmpeg_${Component}_LIBRARIES})
    else()
        set(FFmpeg_FOUND FALSE)
    endif()
endforeach()
# remove duplicated include dirs
list(REMOVE_DUPLICATES FFmpeg_INCLUDE_DIRS)

# parse FFmpeg version
if(TARGET FFmpeg::avutil)
    set(_FFmpegVersionHeader "${FFmpeg_avutil_INCLUDE_DIRS}/libavutil/ffversion.h")
    if (EXISTS "${_FFmpegVersionHeader}")
        file(STRINGS "${_FFmpegVersionHeader}" _FFmpegVersion  REGEX "FFMPEG_VERSION")
        string(REGEX REPLACE ".*\"n?\(.*\)\"" "\\1" FFmpegVersion "${_FFmpegVersion}")
    endif()
endif()

# show message
if (FFmpeg_FOUND)
    message(STATUS "Found FFmpeg (Version: ${FFmpegVersion}), find components: ${FFmpeg_FIND_COMPONENTS}")
    message(STATUS "  Includes: ${FFmpeg_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${FFmpeg_LIBRARIES}")
else()
    if (FFmpeg_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find FFmpeg")
    endif()
endif()

find_package_handle_standard_args(FFmpeg
    REQUIRED_VARS FFmpeg_INCLUDE_DIRS
    VERSION_VAR FFmpegVersion
    HANDLE_COMPONENTS
)