add_subdirectory(gcem)
add_library(BS_thread_pool INTERFACE)
target_include_directories(BS_thread_pool INTERFACE thread-pool/include)
