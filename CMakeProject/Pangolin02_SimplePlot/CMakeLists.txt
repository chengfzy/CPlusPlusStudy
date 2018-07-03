cmake_minimum_required(VERSION 3.2)
set(CMAKE_CXX_STANDARD 14)
project(Pangolin02_SimplePlot)

# library
find_package(Pangolin REQUIRED)                 # Pangolin
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${Pangolin_LIBRARIES})
