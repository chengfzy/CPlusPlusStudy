# Eigen Study
find_package(Eigen3)

if (${Eigen3_FOUND})
    list(APPEND PRIVATE_DEPEND_INCLUDES ${EIGEN3_INCLUDE_DIRS})
    list(APPEND PRIVATE_DEPEND_LIBS ${PRJ_LIB_COMMON})

    add_subdirectory(Eigen01_Basic)
    add_subdirectory(Eigen02_LinearEquationSolver)
    add_subdirectory(Eigen03_Interpolation)
endif ()


