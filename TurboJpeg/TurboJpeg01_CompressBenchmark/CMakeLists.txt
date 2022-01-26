# Compress Benchmark
project(TurboJpeg01_CompressBenchmark)

# libraries
find_package(OpenCV REQUIRED)               # OpenCV
if(OpenCV_FOUND)
    list(APPEND DEPEND_INCLUDES ${OpenCV_INCLUDE_DIRS})
    list(APPEND DEPEND_LIBS ${OpenCV_LIBRARIES})
endif()

# build target
add_executable(${PROJECT_NAME} ${FILE_INCLUDES} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${DEPEND_INCLUDES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${DEPEND_LIBS})