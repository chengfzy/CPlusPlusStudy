# Json Study

# find libraries
find_package(jsoncpp QUIET)       # jsoncpp

if(${jsoncpp_FOUND})
    list(APPEND DEPEND_LIBS ${PRJ_LIB_COMMON} jsoncpp_lib)

    add_subdirectory(jsoncpp01_Basic)
endif()