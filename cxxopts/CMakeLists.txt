# cxxopts Study
find_package(cxxopts)

if (${cxxopts_FOUND})
    list(APPEND PRIVATE_DEPEND_LIBS ${PRJ_LIB_COMMON} cxxopts::cxxopts)

    # build project
    project(cxxoptsStudy)
    add_executable(${PROJECT_NAME} main.cpp)
    target_include_directories(${PROJECT_NAME} PRIVATE ${PRIVATE_DEPEND_INCLUDES})
    target_link_libraries(${PROJECT_NAME} ${PRIVATE_DEPEND_LIBS})
    add_dependencies(${PROJECT_NAME} ${PRJ_LIB_COMMON})
endif ()