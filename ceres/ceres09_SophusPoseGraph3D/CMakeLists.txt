project(ceres09_SophusPoseGraph3D)

# build project
file(GLOB_RECURSE FILE_SRCS ./*.*)
add_executable(${PROJECT_NAME} ${FILE_SRCS})
target_include_directories(${PROJECT_NAME} PRIVATE ${PRIVATE_DEPEND_INCLUDES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${PRIVATE_DEPEND_LIBS})