project(spdlog04_UseInDll)

# build lib
add_library(${PROJECT_NAME}lib SHARED MyLib.h MyLib.cpp)
target_include_directories(${PROJECT_NAME}lib PRIVATE ${DEPEND_INCLUDES})
target_link_libraries(${PROJECT_NAME}lib PRIVATE ${PRJ_LIB_COMMON} ${DEPEND_LIBS})
add_dependencies(${PROJECT_NAME}lib ${PRJ_LIB_COMMON})

# build project
add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${DEPEND_INCLUDES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${PRJ_LIB_COMMON} ${DEPEND_LIBS} ${PROJECT_NAME}lib)
add_dependencies(${PROJECT_NAME} ${PRJ_LIB_COMMON} ${PROJECT_NAME}lib)