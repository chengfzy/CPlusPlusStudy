project(Alglib01_Interpolation)

# build project
set(3RDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty")
file(GLOB_RECURSE FILES_3RDPARTY "${3RDPARTY_DIR}/alglib/*.*")
add_executable(${PROJECT_NAME} main.cpp ${FILES_3RDPARTY})
target_include_directories(${PROJECT_NAME} PRIVATE ${3RDPARTY_DIR})