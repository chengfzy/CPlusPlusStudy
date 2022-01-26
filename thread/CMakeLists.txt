# Thread Study: C++11 thread

find_package(Threads REQUIRED)

# depend libs
list(APPEND DEPEND_LIBS Threads::Threads ${PRJ_LIB_COMMON})

add_subdirectory(thread01_HelloWorld)
add_subdirectory(thread02_Basic)
add_subdirectory(thread03_Mutex)
add_subdirectory(thread04_TimeMutex)
add_subdirectory(thread05_LockGuard)
add_subdirectory(thread06_UniqueLock)
add_subdirectory(thread07_ConditionVariable01)
add_subdirectory(thread08_ConditionVariable02)
add_subdirectory(thread09_Async01)
add_subdirectory(thread10_JobQueue)
add_subdirectory(thread11_PackagedTask)
add_subdirectory(thread12_ComplexPackagedTask)
add_subdirectory(thread13_AtomicBasic)
add_subdirectory(thread20_ComplexProject)
