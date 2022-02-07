# Simple Video Encoder using libx264
project(VideoProcess01_SimpleX264Encoder)

find_package(OpenCV)    # OpenCV
find_package(X264)      # x264
if(${OpenCV_FOUND} AND ${X264_FOUND})
    list(APPEND DEPEND_INCLUDES ${OpenCV_INCLUDE_DIRS} ${X264_INCLUDE_DIRS})
    list(APPEND DEPEND_LIBS ${OpenCV_LIBRARIES} ${X264_LIBRARIES})
    
    # build target
    add_executable(${PROJECT_NAME} ${FILE_INCLUDES} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
    target_include_directories(${PROJECT_NAME} PRIVATE ${DEPEND_INCLUDES})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${DEPEND_LIBS})
endif()