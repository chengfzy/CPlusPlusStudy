project(thread03_Mutex)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ${DEPEND_LIBS})
