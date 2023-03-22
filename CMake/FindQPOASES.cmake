
SET (QPOASES_FOUND FALSE)
find_path(QPOASES_INCLUDE_DIR qpOASES.hpp
    PATHS
        /usr
        /usr/local
    PATH_SUFFIXES
        include
    )

find_library(QPOASES_LIBRARY
    NAMES 
        qpOASES
    PATHS
        /usr
        /usr/local
    PATH_SUFFIXES
        include
    )


IF (QPOASES_INCLUDE_DIR AND QPOASES_LIBRARY)
	SET (QPOASES_FOUND TRUE)
ENDIF (QPOASES_INCLUDE_DIR AND QPOASES_LIBRARY)

mark_as_advanced(QPOASES_INCLUDE_DIR QPOASES_LIBRARY)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(QPOASES
    REQUIRED_VARS
        QPOASES_LIBRARY
        QPOASES_INCLUDE_DIR
)