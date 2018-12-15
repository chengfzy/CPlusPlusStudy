cmake_minimum_required(VERSION 3.2)
project(Sqlite01_Basic)

# SQLiteCpp
find_package(SQLiteCpp REQUIRED)
include_directories(${SQLiteCpp_INCLUDE_DIRS})

# build the target
aux_source_directory(./ DIR_SRCS)
add_executable(${PROJECT_NAME} ${DIR_SRCS})
target_link_libraries(${PROJECT_NAME} ${SQLiteCpp_LIBRARIES})
