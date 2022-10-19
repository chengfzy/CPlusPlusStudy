if(UNIX)
    project(system01_GetNetDeviceInfo)

    # build project
    add_executable(${PROJECT_NAME} main.cpp)
    target_include_directories(${PROJECT_NAME} PRIVATE ${DEPEND_INCLUDES})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${DEPEND_LIBS} ${PRJ_LIB_COMMON})
    add_dependencies(${PROJECT_NAME} ${PRJ_LIB_COMMON})
endif()