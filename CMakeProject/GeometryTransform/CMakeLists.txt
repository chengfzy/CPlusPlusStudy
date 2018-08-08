cmake_minimum_required(VERSION 3.2)
project(GeometryTransform)

# library
find_package(Eigen3 REQUIRED)   # Eigen
find_package(Sophus REQUIRED)   # Sophus
find_package(OpenCV REQUIRED)   # OpenCV
include_directories(${EIGEN3_INCLUDE_DIR} ${SOPHUS_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set(THIRD_LIBS ${THIRD_LIBS} gflags glog ${SOPHUS_LIBRARIES} ${OpenCV_LIBS})

# include common folder
include_directories(../common)

# project
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${THIRD_LIBS})
