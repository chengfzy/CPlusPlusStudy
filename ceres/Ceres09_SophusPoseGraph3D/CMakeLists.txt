cmake_minimum_required(VERSION 3.2)
project(Ceres09_SophusPoseGraph3D)

# Ceres
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
include_directories(${CERES_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS})

# build the target
aux_source_directory(./ DIR_SRCS)
add_executable(${PROJECT_NAME} ${DIR_SRCS})
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})
