cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
project(CPlusPlusStudy)

# cmake modules
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

###############################################################################
# Libraries
###############################################################################
# find packages for basic libraries
find_package(Gflags REQUIRED)                                   # gflags
find_package(Glog REQUIRED)                                     # glog
find_package(fmt REQUIRED)                                      # format
find_package(Eigen3 REQUIRED)                                   # Eigen

# prive dependency include directories and libraries
list(APPEND DEPEND_INCLUDES
    ${GFLAGS_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
list(APPEND DEPEND_LIBS
    ${GFLAGS_LIBRARIES}
    ${GLOG_LIBRARIES}
    fmt::fmt-header-only
)

###############################################################################
# Project Setting
###############################################################################
# default build type 
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

# projects build setting
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# project name
set(PRJ_LIB_COMMON common)    # common library

# project depends
list(APPEND DEPEND_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/common/include)
list(APPEND DEPEND_LIBS)

# projects
add_subdirectory(common)            # common library

add_subdirectory(syntax)            # c++ syntax
add_subdirectory(thread)            # C++11 thread
add_subdirectory(Eigen)             # Eigen study
add_subdirectory(Sophus)            # Sophus study
add_subdirectory(ceres)             # Ceres study
# add_subdirectory(g2o)               # g2o study
add_subdirectory(pcl)               # PCL study
# add_subdirectory(clipper)           # clipper study
add_subdirectory(algorithm)         # algorithm
add_subdirectory(math)              # math    
# add_subdirectory(Pangolin)          # Pangolin study
add_subdirectory(boost)             # boost study
# add_subdirectory(cxxopts)           # cxxopts study
# add_subdirectory(spdlog)            # spdlog study
# add_subdirectory(json)              # json(nlohmann/json) study
# add_subdirectory(jsoncpp)           # jsoncpp study
# add_subdirectory(yaml)              # YAML study
# add_subdirectory(protobuf)          # protobuf study
# add_subdirectory(SqliteCpp)         # SQLiteCpp study
# add_subdirectory(SqliteOrm)         # SQLite ORM study
# add_subdirectory(SqliteModernCpp)   # SqliteModernCpp study
# add_subdirectory(archive)           # libarchive study
# add_subdirectory(httplib)           # httplib
# add_subdirectory(hv)                # libhv
# add_subdirectory(TurboJpeg)         # TurboJpeg study
# add_subdirectory(VideoProcess)      # video process
# add_subdirectory(system)            # system applications
# add_subdirectory(pybind)            # pybind study
# add_subdirectory(others)            # others