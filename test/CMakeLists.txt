add_executable(simple_test test.cpp)
target_link_libraries(simple_test dwbc)

add_executable(thread_test test_thread.cpp)
target_link_libraries(thread_test dwbc)