# PCL Study
find_package(PCL QUIET)

if(${PCL_FOUND})
    list(APPEND DEPEND_INCLUDES ${PCL_INCLUDE_DIRS})
    list(APPEND DEPEND_LIBS ${PCL_LIBRARIES})

    add_subdirectory(pcl01_Basic)
    add_subdirectory(pcl02_PassThroughFilter)
    add_subdirectory(pcl03_VoxelGridFilter)
    add_subdirectory(pcl04_IterativeClosestPoint)
    add_subdirectory(pcl05_IncrementalRegistration)
endif()