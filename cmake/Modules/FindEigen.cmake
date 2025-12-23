#############################################################
#   Find Eigen (wrapper for Eigen3)
#   Written on 2025.12.23.
#############################################################

set(PACKAGE_NAME Eigen)

get_filename_component(this_cmake_file ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
message(STATUS
    "\n-------------------- Finding ${PACKAGE_NAME} in ${this_cmake_file} --------------------------")

# Prefer config-mode packages
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG TRUE)

# Try to locate Eigen (Eigen3 package)
find_package(Eigen3 REQUIRED)

if(Eigen3_FOUND)
    message(STATUS "Eigen3 is found!")
    cmake_print_properties( TARGETS Eigen3::Eigen
    PROPERTIES
        INCLUDE_DIRECTORIES
        INTERFACE_INCLUDE_DIRECTORIES
        IMPORTED_LOCATION_DEBUG
        IMPORTED_IMPLIB_DEBUG
        IMPORTED_LOCATION_RELEASE
        IMPORTED_IMPLIB_RELEASE
        IMPORTED_CONFIGURATIONS
        IMPORTED_LOCATION
        IMPORTED_IMPLIB
    )
else()
    message(FATAL_ERROR "Eigen3 is not found but is required by the project.")
endif()

message(STATUS
    "-------------------- Finding ${PACKAGE_NAME} in ${this_cmake_file} --------------------------\n")
