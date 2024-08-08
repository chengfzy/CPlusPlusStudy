# clipper Study

# find libraries
find_package(clipper REQUIRED)         # clipper
find_package(pmc REQUIRED)             # pmc

if(${clipper_FOUND})
    list(APPEND DEPEND_LIBS ${PRJ_LIB_COMMON} clipper pmc::pmc ${CLIPPER_LIBRARIES} /usr/local/lib/libpmc.so)
    message(STATUS "depends: ${DEPEND_LIBS}")

    add_subdirectory(clipper01_Basic)
endif()