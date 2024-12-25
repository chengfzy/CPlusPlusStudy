# Boost study
find_package(Boost COMPONENTS filesystem date_time serialization system iostreams)

if(${Boost_FOUND})
    message(STATUS "build boost study")
    list(APPEND DEPEND_INCLUDES ${Boost_INCLUDE_DIRS})
    list(APPEND DEPEND_LIBS ${Boost_LIBRARIES} ${PRJ_LIB_COMMON} pthread)

    add_subdirectory(boost01_FileSystem)        # fileSystem
    add_subdirectory(boost02_TimeStudy)         # time
    # add_subdirectory(boost03_Interpolation)     # interpolation, should use boost 1.6
    add_subdirectory(boost04_Tokenizer)         # tokenizer
    add_subdirectory(boost05_LexicalCast)       # lexical cast
    add_subdirectory(boost06_Serialization)     # serialization
    add_subdirectory(boost07_Process)           # process
    add_subdirectory(boost08_CompresssFile)     # compress file

    # geometry
    add_subdirectory(geometry01_ConvexHull)     # convex hull
    
    # asio and beast
    if(${Boost_VERSION} VERSION_GREATER_EQUAL "1.69.0")
        message(STATUS "build boost::asio study")
        add_subdirectory(asio01_Basic)              # basic
        add_subdirectory(asio02_SyncTcpClient)      # synchronous TCP client
        add_subdirectory(asio03_SyncTcpServer)      # synchronous TCP server
        add_subdirectory(asio04_AsyncTcpServer)     # asynchronous TCP server
        add_subdirectory(asio05_SyncUdpClient)      # asynchronous UDP client
        add_subdirectory(asio06_SyncUdpServer)      # synchronous UDP server
        add_subdirectory(asio07_AsyncUdpServer)     # asynchronous UDP server
        add_subdirectory(asio08_AsyncTcpUdpServer)  # asynchronous TCP&UDP server
        add_subdirectory(asio09_MulticastSender)    # multicast sender using UDP
        add_subdirectory(asio10_MulticastReceiver)  # multicast receiver using UDP
        add_subdirectory(asio11_ObtainWebPage)      # asio to obtain web page

        message(STATUS "build boost::beast study")
        add_subdirectory(beast01_SimpleHttpClient)      # simple HTTP client
        add_subdirectory(beast02_SimpleWebSocketClient) # simple WebSocket client
        add_subdirectory(beast03_SyncWebSocketClient)   # synchronous WebSocket client
        add_subdirectory(beast04_SyncWebSocketServer)   # synchronous WebSocket server
        add_subdirectory(beast05_AsyncWebSocketClient)  # asynchronous WebSocket client
        add_subdirectory(beast06_AsyncWebSocketServer)  # asynchronous WebSocket server
        add_subdirectory(beast07_SyncHttpClient)        # synchronous HTTP client
        # add_subdirectory(beast08_SyncHttpServer)        # synchronous HTTP server
        add_subdirectory(beast09_AsyncHttpClient)       # asynchronous HTTP client
        # add_subdirectory(beast10_AsyncHttpServer)       # asynchronous HTTP server
    endif()
endif()