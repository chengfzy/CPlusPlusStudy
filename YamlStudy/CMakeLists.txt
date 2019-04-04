# Yaml Study
project(YamlStudy)

# yaml-cpp
find_package(yaml-cpp)

if (${yaml-cpp_FOUND})
    list(APPEND PRIVATE_DEPEND_INCLUDES ${YAML_CPP_INCLUDE_DIRS})
    list(APPEND PRIVATE_DEPEND_LIBS ${YAML_CPP_LIBRARIES} ${PRJ_LIB_COMMON})

    # build project
    project(YamlStudy)
    add_executable(${PROJECT_NAME} main.cpp)
    target_include_directories(${PROJECT_NAME} PRIVATE ${PRIVATE_DEPEND_INCLUDES})
    target_link_libraries(${PROJECT_NAME} ${PRIVATE_DEPEND_LIBS})
endif ()
