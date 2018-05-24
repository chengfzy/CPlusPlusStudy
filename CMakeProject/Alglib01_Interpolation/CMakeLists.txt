cmake_minimum_required(VERSION 3.2)
project(Alglib01_Interpolation)

# 3rd party library
set(3RDPARTY_DIR "../3rdparty")
include_directories(${3RDPARTY_DIR})
file(GLOB_RECURSE FILES_3RDPARTY "${3RDPARTY_DIR}/alglib/*.*")

add_executable(${PROJECT_NAME} main.cpp ${FILES_3RDPARTY})
