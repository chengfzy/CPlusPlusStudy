# Ceres Study

# find package
find_package(Ceres)

if (${Ceres_FOUND})
    list(APPEND PRIVATE_DEPEND_INCLUDES ${CERES_INCLUDE_DIRS})
    list(APPEND PRIVATE_DEPEND_LIBS ${CERES_LIBRARIES} ${PRJ_LIB_COMMON})

    add_subdirectory(ceres01_HelloWorld)
    add_subdirectory(ceres02_Powell)
    add_subdirectory(ceres03_CurveFitting)
    add_subdirectory(ceres04_BundleAdjustment)
    add_subdirectory(ceres05_libMVBundleAdjuster)
    add_subdirectory(ceres06_RobotPoseMLE)
    add_subdirectory(ceres07_PoseGraph2D)
    add_subdirectory(ceres08_PoseGraph3D)
    add_subdirectory(ceres09_SophusPoseGraph3D)
endif ()