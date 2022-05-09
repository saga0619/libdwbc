PROJECT (DyrosHQP)

CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

add_compile_options(-std=c++17 -Ofast)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake)

find_package(RBDL REQUIRED)
find_package(Eigen3 3.0.0 REQUIRED)
find_package(QPOASES REQUIRED)

include_directories(
    include
    ${RBDL_INCLUDE_DIR} 
    ${EIGEN3_INCLUDE_DIR}
    ${QPOASES_INCLUDE_DIR}
    )

add_library(${PROJECT_NAME} src/dyros_hqp_lib.cpp)

target_link_libraries(${PROJECT_NAME}
${RBDL_LIBRARY}
${RBDL_URDFReader_LIBRARY}
${QPOASES_LIBRARY}
)

# compile test program
add_subdirectory(test)