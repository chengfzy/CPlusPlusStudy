# SQLite Modern CPP Study

# find package
find_package(SqliteModernCpp QUIET)
find_package(Sqlcipher QUIET)

if(${SqliteModernCpp_FOUND} AND ${Sqlcipher_FOUND})
    message(STATUS "Build SQLite Modern Cpp study")
    list(APPEND DEPEND_INCLUDES ${SqliteModernCpp_INCLUDE_DIRS} ${Sqlcipher_INCLUDE_DIRS})
    list(APPEND DEPEND_LIBS  ${PRJ_LIB_COMMON} ${Sqlcipher_LIBRARIES})

    add_subdirectory(SqliteModernCpp01_Basic)
endif()