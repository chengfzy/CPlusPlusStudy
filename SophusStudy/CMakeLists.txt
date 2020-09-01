# Sophus Study
find_package(Eigen3)
find_package(Sophus)

if (${Eigen3_FOUND} AND ${Sophus_FOUND})
    list(APPEND PRIVATE_DEPEND_INCLUDES ${EIGEN3_INCLUDE_DIRS} ${SOPHUS_INCLUDE_DIRS})
    list(APPEND PRIVATE_DEPEND_LIBS ${PRJ_LIB_COMMON})

    # build project
    project(SophusStudy)
    add_executable(${PROJECT_NAME} main.cpp)
    target_include_directories(${PROJECT_NAME} PRIVATE ${PRIVATE_DEPEND_INCLUDES})
    target_link_libraries(${PROJECT_NAME} ${PRIVATE_DEPEND_LIBS})
    add_dependencies(${PROJECT_NAME} ${PRJ_LIB_COMMON})
endif ()