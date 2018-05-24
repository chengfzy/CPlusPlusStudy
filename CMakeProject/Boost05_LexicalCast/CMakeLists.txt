cmake_minimum_required(VERSION 3.2)
project(Boost05_LexicalCast)

# boost: bypass systems path for search another boost folder
set(BOOST_NO_SYSTEM_PATHS TRUE)
set(BOOST_ROOT "/usr/local")
find_package(Boost 1.60 REQUIRED)
if (NOT Boost_FOUND)
    message(FATAL_ERROR Boost (version > 1.60) required)
endif()
include_directories(${Boost_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
