cmake_minimum_required(VERSION 3.2)
project(Eigen01_Basic)

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(${PROJECT_NAME} main.cpp)
