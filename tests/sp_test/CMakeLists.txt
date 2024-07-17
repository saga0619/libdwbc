CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

ADD_EXECUTABLE(dwbc_example redu_dyn_test.cpp)
target_include_directories(dwbc_example PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(dwbc_example dwbc)

ADD_EXECUTABLE(regulation_test regulation_test.cpp)
target_include_directories(regulation_test PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(regulation_test dwbc)

ADD_EXECUTABLE(test_case_writer test_case_writer.cpp)
target_include_directories(test_case_writer PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(test_case_writer dwbc)

ADD_EXECUTABLE(data_confirmation data_confirmation.cpp)
target_include_directories(data_confirmation PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(data_confirmation dwbc)

ADD_EXECUTABLE(jacc_compare jacc_compare.cpp)
target_include_directories(jacc_compare PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(jacc_compare dwbc)

ADD_EXECUTABLE(herzog_test herzog_test.cpp)
target_include_directories(herzog_test PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(herzog_test dwbc)

ADD_EXECUTABLE(dof_comaprison dof_comparison.cpp)
target_include_directories(dof_comaprison PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(dof_comaprison dwbc)
