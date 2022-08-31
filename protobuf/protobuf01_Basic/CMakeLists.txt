project(protobuf01_Basic)

# find .proto files
set(ProtoFileDir ${CMAKE_CURRENT_SOURCE_DIR}/proto)
file(GLOB_RECURSE ProtoFiles ${ProtoFileDir}/*.proto)
# message(STATUS "proto file dir: ${ProtoFileDir}")
message(STATUS "proto files: ${ProtoFiles}")

# # generate cpp file for each file
# foreach(ProtoFile ${ProtoFiles})
#     get_filename_component(FileStem ${ProtoFile} NAME_WE)
#     get_filename_component(FileDir ${ProtoFile} DIRECTORY)
#     message(STATUS "stem: ${FileStem}, directory: ${FileDir}")

#     # get namespace path
#     string(REPLACE "${ProtoFileDir}/" "" NamespaceFolder ${FileDir})
#     # get output dir
#     # set(ProtoCppOutDir ${CMAKE_CURRENT_BINARY_DIR}/${NamespaceFolder})
#     message(STATUS "namespace: ${NamespaceFolder}")
#     message(STATUS "cpp out dir: ${ProtoCppOutDir}")

#     # generate cpp file
#     execute_process(COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} --proto_path=${ProtoFileDir} --cpp_out=${CMAKE_CURRENT_BINARY_DIR} ${ProtoFile})
# endforeach()

# generate cpp file
execute_process(COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} --proto_path=${ProtoFileDir} --cpp_out=${CMAKE_CURRENT_BINARY_DIR} ${ProtoFiles})

# find generated cpp and include
file(GLOB_RECURSE PROTO_INCLUDES ${CMAKE_CURRENT_BINARY_DIR}/*.pb.h)
file(GLOB_RECURSE PROTO_SRCS ${CMAKE_CURRENT_BINARY_DIR}/*.pb.cc)

# # generate cpp
# protobuf_generate_cpp(PROTO_SRCS PROTO_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/Person.proto)
# message(STATUS "generate proto includes: ${PROTO_INCLUDES}")
# message(STATUS "generate proto srcs: ${PROTO_SRCS}")

# build project
add_executable(${PROJECT_NAME} main.cpp ${PROTO_SRCS} ${PROTO_INCLUDES})
target_include_directories(${PROJECT_NAME} PRIVATE ${DEPEND_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(${PROJECT_NAME} PRIVATE ${DEPEND_LIBS})
add_dependencies(${PROJECT_NAME} ${PRJ_LIB_COMMON})