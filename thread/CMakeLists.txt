# Thread Study: C++11 thread

# depend libs
list(APPEND DEPEND_LIBS pthread ${PRJ_LIB_COMMON})

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
add_subdirectory(thread20_ComplexProject)
