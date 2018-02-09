cmake_minimum_required(VERSION 3.2)
project(CPlusPlusStudyCMake)

set(CMAKE_CXX_STANDARD 14)

# cmake modules
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# YAML Study
add_subdirectory(YamlStudy)

# Eigen Study
add_subdirectory(EigenStudy)

# Sophus Study
add_subdirectory(SophusStudy)

# Boost Time Study
add_subdirectory(BoostTimeStudy)
