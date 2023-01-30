project(asio08_AsyncTcpUdpServer)

# build project
add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${DEPEND_INCLUDES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${DEPEND_LIBS})