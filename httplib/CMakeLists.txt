# cpp-httplib Study
find_package(httplib QUIET)

if(${httplib_FOUND})
    list(APPEND DEPEND_LIBS httplib::httplib)

    add_subdirectory(http01_Basic)
    add_subdirectory(http02_WebClient)
    add_subdirectory(http03_Post)
endif()