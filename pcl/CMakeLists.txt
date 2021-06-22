# PCL Study
find_package(PCL)

if (${PCL_FOUND})
    list(APPEND DEPEND_INCLUDES ${PCL_INCLUDE_DIRS})
    list(APPEND DEPEND_LIBS ${PCL_LIBRARIES})

    add_subdirectory(pcl01_Basic)
endif ()