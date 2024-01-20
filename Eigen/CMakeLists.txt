# Eigen Study
find_package(Eigen3 QUIET)

if(${Eigen3_FOUND})
    list(APPEND DEPEND_INCLUDES ${EIGEN3_INCLUDE_DIRS})

    add_subdirectory(Eigen01_Basic)
    add_subdirectory(Eigen02_LinearEquationSolver)
    add_subdirectory(Eigen03_Interpolation)
    add_subdirectory(Eigen04_Format)
endif()