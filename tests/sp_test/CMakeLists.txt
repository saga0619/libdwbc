CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

# ADD_EXECUTABLE(mod_control_test 01_mod_control_test.cpp)
ADD_EXECUTABLE(model_mod_test 02_model_mod_test.cpp)
ADD_EXECUTABLE(cmm_calc_test 03_cmm_calc_test.cpp)

# target_include_directories(mod_control_test PUBLIC
#     ${CMAKE_SOURCE_DIR}/include
#     ${CMAKE_SOURCE_DIR}/tests)
# target_link_libraries(mod_control_test dwbc)

target_include_directories(model_mod_test PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(model_mod_test dwbc)

target_include_directories(cmm_calc_test PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(cmm_calc_test dwbc)

ADD_EXECUTABLE(inversion_test 04_inversion_test.cpp)
target_include_directories(inversion_test PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(inversion_test dwbc)


ADD_EXECUTABLE(lambda_test 05_lambda_calc_test.cpp)
target_include_directories(lambda_test PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(lambda_test dwbc)

ADD_EXECUTABLE(06_final_test 06_final_test.cpp)
target_include_directories(06_final_test PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(06_final_test dwbc)
