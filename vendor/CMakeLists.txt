set(CMAKE_POLICY_VERSION_MINIMUM 3.12) # Workaround until gcem repo fixes cmake minimum version
add_subdirectory(gcem)
add_library(BS_thread_pool INTERFACE)
target_include_directories(BS_thread_pool INTERFACE thread-pool/include)
