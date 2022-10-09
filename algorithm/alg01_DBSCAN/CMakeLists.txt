project(alg01_DBSCAN)

find_package(Boost COMPONENTS filesystem)
find_package(Eigen3 QUIET)
if(${Boost_FOUND} AND ${Eigen3_FOUND})
    list(APPEND DEPEND_INCLUDES ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
    list(APPEND DEPEND_LIBS ${Boost_LIBRARIES} ${PRJ_LIB_COMMON})

    # project files
    file(GLOB_RECURSE FILE_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include/*.*)
    file(GLOB_RECURSE FILE_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.*)

    # app
    add_executable(${PROJECT_NAME} ${FILE_INCLUDES} ${FILE_SRCS})
    target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${DEPEND_INCLUDES})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${DEPEND_LIBS})
    add_dependencies(${PROJECT_NAME} ${PRJ_LIB_COMMON})
endif()