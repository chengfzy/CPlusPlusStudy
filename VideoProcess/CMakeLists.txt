# Some example about video process

# project depends
find_package(cxxopts REQUIRED)                      # cxxopts
find_package(OpenCV REQUIRED)                       # OpenCV
find_package(Boost COMPONENTS filesystem REQUIRED)  # boost
list(APPEND DEPEND_INCLUDES ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
list(APPEND DEPEND_LIBS cxxopts::cxxopts ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

# projects
add_subdirectory(VideoProcess01_SimpleX264Encoder)
add_subdirectory(VideoProcess02_X264Encoder)
add_subdirectory(VideoProcess03_FFmpegEncoder)