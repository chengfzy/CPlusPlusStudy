project(ceres08_PoseGraph3D)

# build project
file(GLOB_RECURSE FILE_SRCS ./*.*)
add_executable(${PROJECT_NAME} ${FILE_SRCS})
target_include_directories(${PROJECT_NAME} PRIVATE ${DEPEND_INCLUDES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${DEPEND_LIBS})