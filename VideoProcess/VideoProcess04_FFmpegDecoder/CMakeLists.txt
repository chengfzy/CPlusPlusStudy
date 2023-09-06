# Video Decoder using ffmpeg
project(VideoProcess04_FFmpegDecoder)

find_package(TurboJpeg QUIET)                                 # turbojpeg
find_package(FFmpeg COMPONENTS avutil avcodec avformat QUIET) # FFmpeg

if(${FFmpeg_FOUND} AND ${TurboJpeg_FOUND})
    list(APPEND DEPEND_INCLUDES ${FFmpeg_INCLUDE_DIRS} ${TurboJpeg_INCLUDE_DIRS})
    list(APPEND DEPEND_LIBS ${FFmpeg_LIBRARIES} ${TurboJpeg_LIBRARIES})

    # project files
    file(GLOB_RECURSE FILE_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include/*.*)
    file(GLOB_RECURSE FILE_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.*)

    # build target
    add_executable(${PROJECT_NAME} ${FILE_INCLUDES} ${FILE_SRCS})
    target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${DEPEND_INCLUDES})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${DEPEND_LIBS})
endif()