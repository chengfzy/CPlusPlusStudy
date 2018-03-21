# find package sophus
# Once done this will define
#  SOPHUS_FOUND - if system found Sophus library
#  SOPHUS_INCLUDE_DIRS - The Sophus include directories
#  SOPHUS_LIBRARIES - The libraries needed to use Sophus
#  SOPHUS_DEFINITIONS - Compiler switches required for using Sophus

find_path(SOPHUS_INCLUDE_DIR NAMES sophus/so3.h
    PATHS
    /usr/include
    /usr/local/include
    /opt/local/include
    DOC "Sophus include directory"
    )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SOPHUS DEFAULT_MSG
    SOPHUS_INCLUDE_DIR)

if (SOPHUS_FOUND)
    set(SOPHUS_INCLUDE_DIRS ${SOPHUS_INCLUDE_DIR})
    set(SOPHUS_DEFINITIONS)
endif()

mark_as_advanced(SOPHUS_INCLUDE_DIR)
