cmake_minimum_required(VERSION 3.2)
project(CPlusPlusStudyCMake)

set(CMAKE_CXX_STANDARD 14)

# cmake modules
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Thread Study: C++11 thread
add_subdirectory(thread01_HelloWorld)
add_subdirectory(thread02_Basic)
add_subdirectory(thread03_Mutex)
add_subdirectory(thread04_TimeMutex)
add_subdirectory(thread05_LockGuard)
add_subdirectory(thread06_UniqueLock)
add_subdirectory(thread07_ConditionVariable01)
add_subdirectory(thread08_ConditionVariable02)


# Boost Study
# Time
add_subdirectory(Boost01_FileSystem)        # fileSystem
add_subdirectory(Boost02_TimeStudy)         # time
#add_subdirectory(Boost03_Interpolation)     # interpolation, should use boost 1.6
add_subdirectory(Boost04_Tokenizer)         # tokenizer
add_subdirectory(Boost05_LexicalCast)       # lexical cast
add_subdirectory(Boost06_Serialization)     # serialization

# YAML Study: conflict with boost in /usr/local
add_subdirectory(YamlStudy)

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
