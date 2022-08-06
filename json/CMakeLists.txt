# Json Study

# find libraries
find_package(nlohmann_json QUIET)         # nlohmann_json

if(${nlohmann_json_FOUND})
    list(APPEND DEPEND_LIBS ${PRJ_LIB_COMMON} nlohmann_json::nlohmann_json)

    add_subdirectory(json01_Basic)
endif()