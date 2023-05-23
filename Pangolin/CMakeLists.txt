# Pangolin Study

# find package
find_package(Pangolin QUIET)

if(${Pangolin_FOUND})
    list(APPEND DEPEND_INCLUDES ${Pangolin_INCLUDE_DIRS})
    list(APPEND DEPEND_LIBS ${Pangolin_LIBRARIES})

    add_subdirectory(Pangolin01_HelloPangolin)
    add_subdirectory(Pangolin02_SimplePlot)
    # add_subdirectory(Pangolin03_SimpleDisplay)
    add_subdirectory(Pangolin04_MultiDisplay)
endif()