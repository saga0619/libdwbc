CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

ADD_EXECUTABLE(dwbc_example2 test_mod2.cpp)
target_include_directories(dwbc_example2 PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(dwbc_example2 dwbc)

ADD_EXECUTABLE(dwbc_example redu_dyn_test.cpp)
target_include_directories(dwbc_example PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(dwbc_example dwbc)

ADD_EXECUTABLE(rot_test rot_test.cpp)
target_include_directories(rot_test PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(rot_test dwbc)
