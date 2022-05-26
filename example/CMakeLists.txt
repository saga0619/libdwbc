CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

project(dwbc_example)

add_compile_options(-std=c++17 -Ofast)

find_package(dwbc REQUIRED)

add_executable(dwbc_example main.cpp)

target_link_libraries(dwbc_example dwbc pthread)