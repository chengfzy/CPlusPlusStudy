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

# prive dependency include directories and libraries
list(APPEND DEPEND_INCLUDES
     ${GFLAGS_INCLUDE_DIRS}
     ${GLOG_INCLUDE_DIRS}
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
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

# projects build setting
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# project name
set(PRJ_LIB_COMMON common)    # common library

# project depends
list(APPEND DEPEND_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/common/include)
list(APPEND DEPEND_LIBS)

# projects
add_subdirectory(common)           # common library

# add_subdirectory(syntax)           # c++ syntax
# add_subdirectory(thread)           # C++11 thread
# add_subdirectory(cxxopts)          # cxxopts study
# add_subdirectory(spdlog)           # spdlog study
add_subdirectory(json)             # json(nlohmann/json) Study
add_subdirectory(jsoncpp)          # jsoncpp Study
# add_subdirectory(yaml)             # YAML Study
# add_subdirectory(Sqlite)           # SQLite Study
# add_subdirectory(boost)            # boost Study
# add_subdirectory(Eigen)            # Eigen Study
# add_subdirectory(ceres)            # Ceres Study
# add_subdirectory(g2o)              # g2o Study
# add_subdirectory(Pangolin)         # Pangolin Study
# add_subdirectory(Sophus)           # Sophus Study
# add_subdirectory(pcl)              # PCL Study
# add_subdirectory(pybind)           # pybind Study
# add_subdirectory(others)           # others