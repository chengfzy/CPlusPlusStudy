cmake_minimum_required(VERSION 3.2)
project(CPlusPlusStudy)

set(CMAKE_CXX_STANDARD 14)

# cmake modules
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# projects build setting
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# inclue common folder
include_directories(common)

# DataSize: print the data size and give a reference when to use const& and value parameters in function
add_subdirectory(DataSize)
# Geometry Transform
add_subdirectory(GeometryTransform)
# alglib
add_subdirectory(Alglib01_Interpolation)

# c++ syntax
add_subdirectory(syntax)

# Thread Study: C++11 thread
add_subdirectory(thread)

# Boost Study
add_subdirectory(boost)

# YAML Study: conflict with boost in /usr/local
add_subdirectory(YamlStudy)

# Eigen Study
add_subdirectory(Eigen)

# Sophus Study
add_subdirectory(SophusStudy)

# Ceres Study
add_subdirectory(ceres)

# Pangolin Study
add_subdirectory(Pangolin)

# SQLite Study
add_subdirectory(Sqlite)