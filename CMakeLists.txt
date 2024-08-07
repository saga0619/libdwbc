cmake_minimum_required(VERSION 2.8...3.15)
PROJECT (dwbc)

add_compile_options(-O3 -std=c++17)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake)


set(CMAKE_INTERFACE_CURRENT_DIR_IN_INTERFACE ON)

OPTION (USE_QPSWIFT "Run with QPSWIFT" OFF)

OPTION (RUN_TEST "Run calculation test of libdwbc" OFF)

OPTION (RUN_BENCHMARK "Run benchmark of libdwbc" OFF)

OPTION (PYBIND11 "Generate python library" OFF)

include(GNUInstallDirs)



IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	MESSAGE(STATUS "Setting build type to 'Release' as none was specified.")
	SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()


find_package(RBDL REQUIRED)
find_package(Eigen3 3.0.0 REQUIRED)
find_package(OsqpEigen REQUIRED)

ADD_DEFINITIONS(-DURDF_DIR=\"${PROJECT_SOURCE_DIR}/tests\")
ADD_DEFINITIONS(-DRESOURCE_DIR=\"${PROJECT_SOURCE_DIR}/tests/cases\")

IF(USE_QPSWIFT)
    find_package(qpSWIFT REQUIRED)
    ADD_DEFINITIONS(-DCOMPILE_QPSWIFT)
    set(QPLIB qpSWIFT::qpSWIFT-static)
ELSE()
    find_package(QPOASES REQUIRED)
    set(QPLIB ${QPOASES_LIBRARY})
ENDIF()

IF(PYBIND11)
    set (PYBIND11_PYTHON_VERSION "3.9")
    find_package(pybind11 CONFIG REQUIRED)
    set(PYBIND_LIB pybind11::module pybind11::lto pybind11::windows_extras)
    file(GLOB pybind_src
    "src/pybind/*.cpp"
    )
    file(GLOB pybind_header
    "include/pybind/*.h"
    )
ENDIF()


include_directories(
    include
    ${QPOASES_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    )

file(GLOB dwbc_src
"src/*.cpp"
)

file(GLOB dwbc_header
"include/*.h"
)

IF (PYBIND11)
    add_library(dwbc MODULE ${dwbc_src} ${pybind_src})
ELSE()
    add_library(dwbc STATIC ${dwbc_src})
ENDIF()

target_link_libraries(dwbc
PUBLIC
${RBDL_LIBRARY}
${RBDL_URDFREADER_LIBRARY}
${QPLIB}
pthread
${PYBIND_LIB}
OsqpEigen::OsqpEigen
)

file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/dwbc-config.cmake" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

INSTALL (TARGETS dwbc
    EXPORT dwbc-export
  	LIBRARY         DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE         DESTINATION ${CMAKE_INSTALL_LIBDIR}
	INCLUDES        DESTINATION include
    )

install(FILES ${dwbc_header} ${pybind_header} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/libdwbc)

include(CMakePackageConfigHelpers)

install(EXPORT dwbc-export
    FILE dwbc-targets.cmake
    DESTINATION share/cmake/dwbc/
    EXPORT_LINK_INTERFACE_LIBRARIES
    )

install(EXPORT dwbc-export
    FILE dwbc-config.cmake
    DESTINATION share/cmake/dwbc/
    EXPORT_LINK_INTERFACE_LIBRARIES
    )

export(
    EXPORT dwbc-export
    FILE "${CMAKE_CURRENT_BINARY_DIR}/dwbc-targets.cmake"
)

# compile test program
IF (NOT PYBIND11)
    IF(RUN_TEST)
        add_subdirectory(tests)
    ENDIF()

    add_subdirectory(tests/sp_test)
ENDIF()

IF (PYBIND11)
    pybind11_extension(${PROJECT_NAME})
    pybind11_strip(${PROJECT_NAME})

    set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET "hidden"
                                         CUDA_VISIBILITY_PRESET "hidden")
ENDIF()
