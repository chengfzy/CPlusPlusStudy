project(syntax06_dataConversion)

# build project
add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${DEPEND_INCLUDES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${PRJ_LIB_COMMON} ${DEPEND_LIBS})
add_dependencies(${PROJECT_NAME} ${PRJ_LIB_COMMON})