cmake_minimum_required(VERSION 3.2)
project(CPlusPlusStudyCMake)

set(CMAKE_CXX_STANDARD 14)

# cmake modules
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Boost Study
# Time
add_subdirectory(Boost01_FileSystem)        # fileSystem
add_subdirectory(Boost02_TimeStudy)         # time
add_subdirectory(Boost03_Interpolation)     # interpolation
add_subdirectory(Boost04_Tokenizer)         # tokenizer
add_subdirectory(Boost05_LexicalCast)       # lexical cast
add_subdirectory(Boost06_Serialization)     # serialization

# YAML Study: conflict with boost in /usr/local
#add_subdirectory(YamlStudy)

# Eigen Study
add_subdirectory(Eigen01_Basic)
add_subdirectory(Eigen02_LinearEquationSolver)
add_subdirectory(Eigen03_Interpolation)

# Sophus Study
add_subdirectory(SophusStudy)

# Geometry Transform
add_subdirectory(GeometryTransform)

# Ceres Study
add_subdirectory(Ceres01_HelloWorld)
add_subdirectory(Ceres02_Powell)
add_subdirectory(Ceres03_CurveFitting)
add_subdirectory(Ceres04_BundleAdjustment)

# alglib
add_subdirectory(Alglib01_Interpolation)
