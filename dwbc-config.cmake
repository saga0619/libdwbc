include(CMakeFindDependencyMacro)

# All find_package we used in CMakeLists.txt should also be there
find_dependency(Eigen3 REQUIRED)
find_dependency(qpOASES REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/dwbc-targets.cmake")