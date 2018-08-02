cmake_minimum_required(VERSION 3.2)
project(CPlusPlusStudyCMake)

set(CMAKE_CXX_STANDARD 14)

# cmake modules
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

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

# Geometry Transform
add_subdirectory(GeometryTransform)

# Ceres Study
add_subdirectory(ceres)

# alglib
add_subdirectory(Alglib01_Interpolation)

# Pangolin Study
add_subdirectory(Pangolin)