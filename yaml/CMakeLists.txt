# Yaml Study
project(YamlStudy)

# yaml-cpp
find_package(yaml-cpp QUIET)

if(${yaml-cpp_FOUND})
    list(APPEND DEPEND_INCLUDES ${YAML_CPP_INCLUDE_DIRS})
    list(APPEND DEPEND_LIBS ${YAML_CPP_LIBRARIES} ${PRJ_LIB_COMMON})

    # build project
    project(YamlStudy)
    add_executable(${PROJECT_NAME} main.cpp)
    target_include_directories(${PROJECT_NAME} PRIVATE ${DEPEND_INCLUDES})
    target_link_libraries(${PROJECT_NAME} ${DEPEND_LIBS})
    add_dependencies(${PROJECT_NAME} ${PRJ_LIB_COMMON})
endif()
