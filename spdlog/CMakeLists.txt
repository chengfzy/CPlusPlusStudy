# spadlog Study
find_package(spdlog QUIET)

if(${spdlog_FOUND})
    # list(APPEND DEPEND_LIBS spdlog::spdlog)
    list(APPEND DEPEND_LIBS spdlog::spdlog_header_only)

    add_subdirectory(spdlog01_Basic)
    add_subdirectory(spdlog02_MultiSink)
    add_subdirectory(spdlog03_CustomerFormat)
    add_subdirectory(spdlog04_UseInDll)
endif()