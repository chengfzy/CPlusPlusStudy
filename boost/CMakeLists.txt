# Boost study
find_package(Boost COMPONENTS filesystem date_time serialization system iostreams)

if(${Boost_FOUND})
    list(APPEND DEPEND_INCLUDES ${Boost_INCLUDE_DIRS})
    list(APPEND DEPEND_LIBS ${Boost_LIBRARIES} ${PRJ_LIB_COMMON} pthread)

    add_subdirectory(boost01_FileSystem)        # fileSystem
    add_subdirectory(boost02_TimeStudy)         # time
    add_subdirectory(boost03_Interpolation)     # interpolation, should use boost 1.6
    add_subdirectory(boost04_Tokenizer)         # tokenizer
    add_subdirectory(boost05_LexicalCast)       # lexical cast
    add_subdirectory(boost06_Serialization)     # serialization
    if(${Boost_VERSION} VERSION_GREATER_EQUAL 106901)
        add_subdirectory(boost07_MulticastSender)   # multicast sender
        add_subdirectory(boost08_MulticastReceiver) # multicast receiver
    endif()
    add_subdirectory(boost09_ObtainWebPage)     # asio to obtain web page
    add_subdirectory(boost10_Process)           # process
    add_subdirectory(boost11_CompresssFile)     # compress file
endif()