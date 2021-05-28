# spadlog Study
find_package(spdlog)

if (${spdlog_FOUND})
    list(APPEND DEPEND_LIBS spdlog::spdlog)

    add_subdirectory(spdlog01_Basic)
    add_subdirectory(spdlog02_MultiSink)
    add_subdirectory(spdlog03_CustomerFormat)
endif ()