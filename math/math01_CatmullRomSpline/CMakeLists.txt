# Curve Fitting using Catmull-Rom Spline

# find package
find_package(cxxopts QUIET)
find_package(Ceres QUIET)
find_package(Eigen3 QUIET)
find_package(
    Boost
    COMPONENTS filesystem
    QUIET)

if(${cxxopts_FOUND}
   AND ${Ceres_FOUND}
   AND ${Eigen3_FOUND}
   AND ${Boost_FOUND})
    list(APPEND DEPEND_INCLUDES ${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
    list(APPEND DEPEND_LIBS Ceres::ceres ${Boost_LIBRARIES} ${PRJ_LIB_COMMON})

    # build project
    project(math01_CatmullRomSpline)
    file(GLOB_RECURSE FILE_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include/*.*)
    file(GLOB_RECURSE FILE_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.*)
    add_executable(${PROJECT_NAME} ${FILE_INCLUDES} ${FILE_SRCS})
    target_include_directories(${PROJECT_NAME} PRIVATE ${DEPEND_INCLUDES} ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${DEPEND_LIBS})
endif()
