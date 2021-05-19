project(ceres11_Pose2DEstimation)

# build project
file(GLOB_RECURSE FILE_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include/*.*)
file(GLOB_RECURSE FILE_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.*)
add_executable(${PROJECT_NAME} ${FILE_INCLUDES} ${FILE_SRCS})
target_include_directories(${PROJECT_NAME} PRIVATE ${DEPEND_INCLUDES} ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} PRIVATE ${DEPEND_LIBS})