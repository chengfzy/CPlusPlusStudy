# libhv study

# find package
find_package(libhv QUIET)
find_package(OpenSSL QUIET)

if(${libhv_FOUND} AND ${OPENSSL_FOUND})
    message(STATUS "Build libhv study")
    list(APPEND DEPEND_LIBS ${PRJ_LIB_COMMON} hv OpenSSL::SSL OpenSSL::Crypto)

    add_subdirectory(hv01_TcpServer)
    add_subdirectory(hv02_TcpClient)
    add_subdirectory(hv03_HttpServer)
    add_subdirectory(hv04_WebSocketServer)
    add_subdirectory(hv05_WebSocketClient)
endif()