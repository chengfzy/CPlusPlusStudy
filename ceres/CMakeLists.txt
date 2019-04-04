# Ceres Study

# find package
find_package(Ceres)

if (${Ceres_FOUND})
    list(APPEND PRIVATE_DEPEND_INCLUDES ${CERES_INCLUDE_DIRS})
    list(APPEND PRIVATE_DEPEND_LIBS ${CERES_LIBRARIES} ${PRJ_LIB_COMMON})

    add_subdirectory(Ceres01_HelloWorld)
    add_subdirectory(Ceres02_Powell)
    add_subdirectory(Ceres03_CurveFitting)
    add_subdirectory(Ceres04_BundleAdjustment)
    add_subdirectory(Ceres05_libMVBundleAdjuster)
    add_subdirectory(Ceres06_RobotPoseMLE)
    add_subdirectory(Ceres07_PoseGraph2D)
    add_subdirectory(Ceres08_PoseGraph3D)
    add_subdirectory(Ceres09_SophusPoseGraph3D)
endif ()