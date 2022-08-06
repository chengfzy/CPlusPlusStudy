# cxxopts Study
find_package(cxxopts QUIET)

if(${cxxopts_FOUND})
    list(APPEND DEPEND_LIBS cxxopts::cxxopts)

    # build project
    project(cxxoptsStudy)
    add_executable(${PROJECT_NAME} main.cpp)
    target_include_directories(${PROJECT_NAME} PRIVATE ${DEPEND_INCLUDES})
    target_link_libraries(${PROJECT_NAME} ${DEPEND_LIBS} ${PRJ_LIB_COMMON})
    add_dependencies(${PROJECT_NAME} ${PRJ_LIB_COMMON})
endif()