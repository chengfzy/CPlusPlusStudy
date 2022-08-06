# pybind11 Study

# find package
find_package(pybind11 QUIET)

if(${pybind11_FOUND})
    list(APPEND DEPEND_INCLUDES ${pybind11_INCLUDE_DIRS})
    list(APPEND DEPEND_LIBS ${pybind11_LIBRARIES})

    add_subdirectory(pybind01_Basic)
    add_subdirectory(pybind02_Class)
endif()