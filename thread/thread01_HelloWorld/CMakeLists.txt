cmake_minimum_required(VERSION 3.2)
project(thread01_HelloWorld)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} pthread)
