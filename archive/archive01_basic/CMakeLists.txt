project(archive01_basic)


find_package(Boost COMPONENTS filesystem)
list(APPEND DEPEND_INCLUDES ${Boost_INCLUDE_DIRS})
list(APPEND DEPEND_LIBS ${Boost_LIBRARIES})

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${DEPEND_INCLUDES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${DEPEND_LIBS})