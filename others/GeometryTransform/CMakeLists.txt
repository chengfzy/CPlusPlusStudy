# find package
find_package(Eigen3 QUIET)
find_package(Sophus QUIET)
find_package(OpenCV QUIET)

if(${Eigen3_FOUND} AND ${Sophus_FOUND} AND ${OpenCV_FOUND})
    list(APPEND DEPEND_INCLUDES ${EIGEN3_INCLUDE_DIR} ${SOPHUS_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
    list(APPEND DEPEND_LIBS ${SOPHUS_LIBRARIES} ${OpenCV_LIBS} ${PRJ_LIB_COMMON})

    project(GeometryTransform)
    # build project
    add_executable(${PROJECT_NAME} main.cpp)
    target_include_directories(${PROJECT_NAME} PRIVATE ${DEPEND_INCLUDES})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${DEPEND_LIBS})
    add_dependencies(${PROJECT_NAME} ${PRJ_LIB_COMMON})
endif()