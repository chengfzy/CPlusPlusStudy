# find package
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)

if (${Eigen3_FOUND} AND ${Sophus_FOUND} AND ${OpenCV_FOUND})
    list(APPEND PRIVATE_DEPEND_INCLUDES ${EIGEN3_INCLUDE_DIR} ${SOPHUS_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
    list(APPEND PRIVATE_DEPEND_LIBS ${SOPHUS_LIBRARIES} ${OpenCV_LIBS} ${PRJ_LIB_COMMON})

    project(GeometryTransform)
    # build project
    add_executable(${PROJECT_NAME} main.cpp)
    target_include_directories(${PROJECT_NAME} PRIVATE ${PRIVATE_DEPEND_INCLUDES})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${PRIVATE_DEPEND_LIBS})
endif ()