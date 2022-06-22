CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

project(dwbc_example)

add_compile_options(-std=c++17 -Ofast)

find_package(dwbc REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(dwbc_example test.cpp)

target_link_libraries(dwbc_example dwbc pthread Eigen3::Eigen)
