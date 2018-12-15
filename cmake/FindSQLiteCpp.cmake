# Find package module for SQLiteCPP library.
#
# The following variables are set by this module:
#
#   SQLiteCpp_FOUND: TRUE if SQLiteCpp is found.
#   SQLiteCpp_INCLUDE_DIRS: Include directories for SQLiteCpp.
#   SQLiteCpp_LIBRARIES: Libraries required to link SQLiteCpp.
#
# The following variables control the behavior of this module:
#
# SQLiteCpp_INCLUDE_DIR_HINTS: List of additional directories in which to search for SQLiteCpp includes.
# SQLiteCpp_LIBRARY_DIR_HINTS: List of additional directories in which to search for SQLiteCpp libraries.

set(SQLiteCpp_INCLUDE_DIR_HINTS "" CACHE PATH "SQLiteCpp include directory")
set(SQLiteCpp_LIBRARY_DIR_HINTS "" CACHE PATH "SQLiteCpp library directory")

unset(SQLiteCpp_FOUND)
unset(SQLiteCpp_INCLUDE_DIRS)
unset(SQLiteCpp_LIBRARIES)

include(FindPackageHandleStandardArgs)

list(APPEND SQLiteCpp_CHECK_INCLUDE_DIRS
        /usr/local/include
        /usr/local/homebrew/include
        /opt/local/var/macports/software
        /opt/local/include
        /usr/include)
list(APPEND SQLiteCpp_CHECK_PATH_SUFFIXES
        SQLiteCpp/include)

list(APPEND SQLiteCpp_CHECK_LIBRARY_DIRS
        /usr/local/lib
        /usr/local/homebrew/lib
        /opt/local/lib
        /usr/lib)
list(APPEND SQLiteCpp_CHECK_LIBRARY_SUFFIXES)

list(APPEND Sqlite3_CHECK_LIBRARY_DIRS
        /usr/local/lib
        /usr/local/homebrew/lib
        /opt/local/lib
        /usr/lib)
list(APPEND Sqlite3_CHECK_LIBRARY_SUFFIXES)

find_path(SQLiteCpp_INCLUDE_DIRS
        NAMES
        SQLiteCpp/SQLiteCpp.h
        PATHS
        ${SQLiteCpp_INCLUDE_DIR_HINTS}
        ${SQLiteCpp_CHECK_INCLUDE_DIRS}
        PATH_SUFFIXES
        ${SQLiteCpp_CHECK_PATH_SUFFIXES})
find_library(SQLiteCpp_LIBRARIES
        NAMES
        SQLiteCpp
        libSQLiteCpp
        PATHS
        ${SQLiteCpp_LIBRARY_DIR_HINTS}
        ${SQLiteCpp_CHECK_LIBRARY_DIRS}
        PATH_SUFFIXES
        ${SQLiteCpp_CHECK_LIBRARY_SUFFIXES})
find_library(Sqlite3_LIBRARIES
        NAMES
        sqlite3
        libsqlite3
        PATHS
        ${Sqlite3_CHECK_LIBRARY_DIRS}
        PATH_SUFFIXES
        ${Sqlite3_CHECK_LIBRARY_SUFFIXES})

if (SQLiteCpp_INCLUDE_DIRS AND SQLiteCpp_LIBRARIES AND Sqlite3_LIBRARIES)
    set(SQLiteCpp_FOUND TRUE)
    set(SQLiteCpp_LIBRARIES ${SQLiteCpp_LIBRARIES} ${Sqlite3_LIBRARIES})
    message(STATUS "Found SQLiteCpp")
    message(STATUS "  Includes: ${SQLiteCpp_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${SQLiteCpp_LIBRARIES}")
else ()
    if (SQLiteCpp_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find SQLiteCpp")
    endif ()
endif ()
