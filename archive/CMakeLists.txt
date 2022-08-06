# liarchive Study
find_package(LibArchive QUIET)

if(${LibArchive_FOUND})
list(APPEND DEPEND_INCLUDES ${LibArchive_INCLUDE_DIRS})
    list(APPEND DEPEND_LIBS ${LibArchive_LIBRARIES})

    add_subdirectory(archive01_basic)
endif()