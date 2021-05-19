project(${PRJ_LIB_COMMON})

# build project
file(GLOB_RECURSE FILE_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include/*.*)
file(GLOB_RECURSE FILE_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.*)
add_library(${PROJECT_NAME} SHARED ${FILE_INCLUDES} ${FILE_SRCS})
target_include_directories(${PROJECT_NAME}
                           PUBLIC
                           $<INSTALL_INTERFACE:include>
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           ${PUBLIC_DEPEND_INCLUDES}
                           PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/src
                           ${DEPEND_INCLUDES}
                           )
target_link_libraries(${PROJECT_NAME}
                      PRIVATE ${DEPEND_LIBS})
set_target_properties(${PROJECT_NAME} PROPERTIES
                      LINKER_LANGUAGE CXX)