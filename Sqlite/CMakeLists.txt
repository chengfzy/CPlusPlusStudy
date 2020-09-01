# SQLite Study

# find package
find_package(SQLiteCpp)

if (${SQLiteCpp_FOUND})
    list(APPEND PRIVATE_DEPEND_INCLUDES ${SQLiteCpp_INCLUDE_DIRS})
    list(APPEND PRIVATE_DEPEND_LIBS ${SQLiteCpp_LIBRARIES} ${PRJ_LIB_COMMON})

    # build project
    project(Sqlite01_Basic)
    add_executable(${PROJECT_NAME} main.cpp)
    target_include_directories(${PROJECT_NAME} PRIVATE ${PRIVATE_DEPEND_INCLUDES})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${PRIVATE_DEPEND_LIBS})
    add_dependencies(${PROJECT_NAME} ${PRJ_LIB_COMMON})
endif ()