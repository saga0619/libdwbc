PROJECT (dwbc)

CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

add_compile_options(-std=c++17 -Ofast)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake)


set(CMAKE_INTERFACE_CURRENT_DIR_IN_INTERFACE ON)
include(GNUInstallDirs)

IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	MESSAGE(STATUS "Setting build type to 'Release' as none was specified.")
	SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()

find_package(RBDL REQUIRED)
find_package(Eigen3 3.0.0 REQUIRED)
find_package(QPOASES REQUIRED)
find_package(qpSWIFT)

include_directories(
    include
    ${QPOASES_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    )


get_directory_property (inc_dirs INCLUDE_DIRECTORIES)

#message(STATUS "eigen dir : ${QPOASES_INCLUDE_DIR}")

message(STATUS "include dir : ${inc_dirs}")

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
${QPOASES_LIBRARY}
PRIVATE
qpSWIFT::qpSWIFT-static
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

export(
    EXPORT dwbc-export
    FILE "${CMAKE_CURRENT_BINARY_DIR}/dwbc-targets.cmake"
)

# compile test program
add_subdirectory(test)


