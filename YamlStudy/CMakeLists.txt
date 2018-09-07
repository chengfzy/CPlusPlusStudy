cmake_minimum_required(VERSION 3.2)
project(YamlStudy)

# yaml-cpp
find_package(yaml-cpp)
include_directories(${YAML_CPP_INCLUDE_DIRS})


add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})
