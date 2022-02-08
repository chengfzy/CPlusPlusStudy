# Video Encoder using libx264
project(VideoProcess02_X264Encoder)

find_package(OpenCV)                        # OpenCV
find_package(X264)                          # x264
find_package(Boost COMPONENTS filesystem)   # boost
find_package(TurboJpeg)                     # turbojpeg

if(${OpenCV_FOUND} AND ${X264_FOUND} AND ${Boost_FOUND} AND ${TurboJpeg_FOUND})
    list(APPEND DEPEND_INCLUDES ${OpenCV_INCLUDE_DIRS} ${X264_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${TurboJpeg_INCLUDE_DIRS})
    list(APPEND DEPEND_LIBS ${OpenCV_LIBRARIES} ${X264_LIBRARIES} ${Boost_LIBRARIES} ${TurboJpeg_LIBRARIES})
    
    # project files
    file(GLOB_RECURSE FILE_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include/*.*)
    file(GLOB_RECURSE FILE_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.*)

    # build target
    add_executable(${PROJECT_NAME} ${FILE_INCLUDES} ${FILE_SRCS})
    target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${DEPEND_INCLUDES})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${DEPEND_LIBS})
endif()