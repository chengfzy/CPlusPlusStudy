cmake_minimum_required(VERSION 3.2)
project(Boost01_FileSystem)

find_package(Boost REQUIRED COMPONENTS filesystem date_time)      # boost
include_directories(${Boost_INCLUDE_DIRS})

## boost: bypass systems path for search another boost folder
#set(BOOST_NO_SYSTEM_PATHS TRUE)
#set(BOOST_ROOT "/usr/local")
##set(BOOST_INCLUDEDIR "${BOOST_ROOT}/include")
##set(BOOST_LIBRARYDIR "${BOOST_ROOT}/lib")
#find_package(Boost 1.67 REQUIRED COMPONENTS filesystem)
#if (NOT Boost_FOUND)
#    message(FATAL_ERROR Boost (version > 1.60) required)
#endif()
#include_directories(${Boost_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
