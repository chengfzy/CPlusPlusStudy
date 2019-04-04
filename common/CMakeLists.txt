project(${PRJ_LIB_COMMON})

# build project
file(GLOB_RECURSE FILE_SRCS ./src/*.*)
add_library(${PROJECT_NAME} STATIC ${FILE_SRCS})
target_include_directories(${PROJECT_NAME}
        PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${PRIVATE_DEPEND_INCLUDES}
        )