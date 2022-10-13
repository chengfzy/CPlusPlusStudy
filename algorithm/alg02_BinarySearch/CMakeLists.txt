project(alg02_BinarySearch)

# project files
file(GLOB_RECURSE FILE_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include/*.*)
file(GLOB_RECURSE FILE_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.*)

# app
add_executable(${PROJECT_NAME} ${FILE_INCLUDES} ${FILE_SRCS})
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${DEPEND_INCLUDES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${DEPEND_LIBS} ${PRJ_LIB_COMMON})
add_dependencies(${PROJECT_NAME} ${PRJ_LIB_COMMON})