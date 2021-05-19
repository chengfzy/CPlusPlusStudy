project(thread05_LockGuard)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ${DEPEND_LIBS})
