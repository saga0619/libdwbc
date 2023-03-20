PROJECT (dwbc)

CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

add_compile_options(-std=c++17 -O -fPIC)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake)


set(CMAKE_INTERFACE_CURRENT_DIR_IN_INTERFACE ON)
set(USE_QPSWIFT OFF)
set(CHECK_DATA OFF)
include(GNUInstallDirs)

IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	MESSAGE(STATUS "Setting build type to 'Release' as none was specified.")
	SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()


find_package(RBDL REQUIRED)
find_package(Eigen3 3.0.0 REQUIRED)

ADD_DEFINITIONS(-DURDF_DIR=\"${PROJECT_SOURCE_DIR}/test\")
ADD_DEFINITIONS(-DRESOURCE_DIR=\"${PROJECT_SOURCE_DIR}/test/Matrix\")

IF(CHECK_DATA)
    ADD_DEFINITIONS(-DCHECKDATA)
ENDIF()
IF(USE_QPSWIFT)
    find_package(qpSWIFT REQUIRED)
    ADD_DEFINITIONS(-DCOMPILE_QPSWIFT)
    set(QPLIB qpSWIFT::qpSWIFT-static)
ELSE()
    find_package(QPOASES REQUIRED)
    set(QPLIB ${QPOASES_LIBRARY})
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

add_library(dwbc ${dwbc_src})

target_link_libraries(dwbc
PUBLIC
${RBDL_LIBRARY}
${RBDL_URDFREADER_LIBRARY}
${QPLIB}
pthread
)

file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/dwbc-config.cmake" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

INSTALL (TARGETS dwbc
    EXPORT dwbc-export
  	LIBRARY         DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE         DESTINATION ${CMAKE_INSTALL_LIBDIR}
	INCLUDES        DESTINATION include
    )

install(FILES ${dwbc_header} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/dwbc)

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
add_subdirectory(test)


