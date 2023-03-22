CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

SET(TEST_SRCS
    dwbc_test.cpp
    catch_amalgamated.cpp
)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

ADD_EXECUTABLE(dwbc_tests ${TEST_SRCS})

target_include_directories(dwbc_tests PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/tests)

TARGET_LINK_LIBRARIES(dwbc_tests
    dwbc
)

OPTION (RUN_AUTOMATIC_TESTS "Run automatic tests" ON)

IF(RUN_AUTOMATIC_TESTS)
    ADD_CUSTOM_COMMAND(TARGET dwbc_tests POST_BUILD
        COMMAND ./tests/dwbc_tests
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Running automatic tests"
    )
ENDIF(RUN_AUTOMATIC_TESTS)