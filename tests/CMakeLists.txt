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


IF(RUN_BENCHMARK)
    ADD_CUSTOM_COMMAND(TARGET dwbc_tests POST_BUILD
        COMMAND ./tests/dwbc_tests -v high
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Running automatic tests"
    )
ELSE()
    ADD_CUSTOM_COMMAND(TARGET dwbc_tests POST_BUILD
    COMMAND ./tests/dwbc_tests -v high --skip-benchmarks
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running automatic tests"
    )
ENDIF(RUN_BENCHMARK)
