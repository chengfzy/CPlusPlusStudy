cmake_minimum_required(VERSION 3.2)
project(CPlusPlusStudyCMake)

set(CMAKE_CXX_STANDARD 14)

# cmake modules
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Boost Time Study
add_subdirectory(BoostTimeStudy)

# Boost FileSystem Study
add_subdirectory(BoostFileSystem)

# YAML Study
add_subdirectory(YamlStudy)

# Eigen Study
add_subdirectory(EigenStudy)

# Sophus Study
add_subdirectory(SophusStudy)

# Ceres Study
add_subdirectory(Ceres01_HelloWorld)
add_subdirectory(Ceres02_Powell)
add_subdirectory(Ceres03_CurveFitting)
add_subdirectory(Ceres04_BundleAdjustment)
