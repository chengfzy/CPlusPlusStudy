# SQLite ORM Study

# find package
find_package(SqliteOrm QUIET)

if(${SqliteOrm_FOUND})
    message(STATUS "Build SQLite ORM study")
    list(APPEND DEPEND_LIBS sqlite_orm::sqlite_orm ${PRJ_LIB_COMMON})

    add_subdirectory(SqliteOrm01_Basic)
endif()