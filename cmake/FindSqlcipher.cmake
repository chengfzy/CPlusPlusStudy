# Find package module for Sqlcipher library.
#
# The following variables are set by this module:
#
#   Sqlcipher_FOUND: TRUE if Sqlcipher is found.
#   Sqlcipher_INCLUDE_DIRS: Include directories for Sqlcipher.
#   Sqlcipher_LIBRARIES: Libraries required to link Sqlcipher.
#
# The following variables control the behavior of this module:
#
# Sqlcipher_INCLUDE_DIR_HINTS: List of additional directories in which to search for Sqlcipher includes.
# Sqlcipher_LIBRARY_DIR_HINTS: List of additional directories in which to search for Sqlcipher libraries.

set(Sqlcipher_INCLUDE_DIR_HINTS "" CACHE PATH "Sqlcipher include directory")
set(Sqlcipher_LIBRARY_DIR_HINTS "" CACHE PATH "Sqlcipher library directory")

unset(Sqlcipher_FOUND)
unset(Sqlcipher_INCLUDE_DIRS)
unset(Sqlcipher_LIBRARIES)

include(FindPackageHandleStandardArgs)

list(APPEND Sqlcipher_CHECK_INCLUDE_DIRS
        /usr/local/include
        /usr/local/homebrew/include
        /opt/local/var/macports/software
        /opt/local/include
        /usr/include)
list(APPEND Sqlcipher_CHECK_PATH_SUFFIXES)

list(APPEND Sqlcipher_CHECK_LIBRARY_DIRS
        /usr/local/lib
        /usr/local/homebrew/lib
        /opt/local/lib
        /usr/lib)
list(APPEND Sqlcipher_CHECK_LIBRARY_SUFFIXES)

list(APPEND Sqlite3_CHECK_LIBRARY_DIRS
        /usr/local/lib
        /usr/local/homebrew/lib
        /opt/local/lib
        /usr/lib)
list(APPEND Sqlite3_CHECK_LIBRARY_SUFFIXES)

find_path(Sqlcipher_INCLUDE_DIRS
        NAMES
        sqlcipher/sqlite3.h
        PATHS
        ${Sqlcipher_INCLUDE_DIR_HINTS}
        ${Sqlcipher_CHECK_INCLUDE_DIRS}
        PATH_SUFFIXES
        ${Sqlcipher_CHECK_PATH_SUFFIXES})
find_library(Sqlcipher_LIBRARIES
        NAMES
        sqlcipher
        libSqlcipher
        PATHS
        ${Sqlcipher_LIBRARY_DIR_HINTS}
        ${Sqlcipher_CHECK_LIBRARY_DIRS}
        PATH_SUFFIXES
        ${Sqlcipher_CHECK_LIBRARY_SUFFIXES})

if (Sqlcipher_INCLUDE_DIRS AND Sqlcipher_LIBRARIES)
    set(Sqlcipher_FOUND TRUE)
    message(STATUS "Found Sqlcipher")
    message(STATUS "  Includes: ${Sqlcipher_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${Sqlcipher_LIBRARIES}")
else ()
    if (Sqlcipher_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find Sqlcipher")
    endif ()
endif ()
