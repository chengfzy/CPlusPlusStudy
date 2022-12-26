# Find package module for SqliteModernCpp library.
#
# The following variables are set by this module:
#
#   SqliteModernCpp_FOUND: TRUE if SqliteModernCpp is found.
#   SqliteModernCpp_INCLUDE_DIRS: Include directories for SqliteModernCpp.
#
# The following variables control the behavior of this module:
#  
# SqliteModernCpp_INCLUDE_DIR_HINTS: List of additional directories in which to search for SqliteModernCpp includes.

set(SqliteModernCpp_INCLUDE_DIR_HINTS "" CACHE PATH "SqliteModernCpp include directory")

unset(SqliteModernCpp_FOUND)
unset(SqliteModernCpp_INCLUDE_DIRS)

include(FindPackageHandleStandardArgs)

list(APPEND SqliteModernCpp_CHECK_INCLUDE_DIRS
        /usr/local/include
        /usr/local/homebrew/include
        /opt/local/var/macports/software
        /opt/local/include
        /usr/include)
list(APPEND SqliteModernCpp_CHECK_PATH_SUFFIXES)

find_path(SqliteModernCpp_INCLUDE_DIRS
        NAMES
        sqlite_modern_cpp.h
        PATHS
        ${SqliteModernCpp_INCLUDE_DIR_HINTS}
        ${SqliteModernCpp_CHECK_INCLUDE_DIRS}
        PATH_SUFFIXES
        ${SqliteModernCpp_CHECK_PATH_SUFFIXES})

if (SqliteModernCpp_INCLUDE_DIRS)
    set(SqliteModernCpp_FOUND TRUE)
    message(STATUS "Found SqliteModernCpp")
    message(STATUS "  Includes: ${SqliteModernCpp_INCLUDE_DIRS}")
else ()
    if (SqliteModernCpp_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find SqliteModernCpp")
    endif ()
endif ()
