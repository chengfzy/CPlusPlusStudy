# protobuf Study

# find libraries
find_package(Protobuf QUIET)         # protobuf

if(${Protobuf_FOUND})
    list(APPEND DEPEND_INCLUDES ${Protobuf_INCLUDE_DIRS})
    list(APPEND DEPEND_LIBS ${PRJ_LIB_COMMON} ${Protobuf_LIBRARIES})

    add_subdirectory(protobuf01_Basic)
endif()