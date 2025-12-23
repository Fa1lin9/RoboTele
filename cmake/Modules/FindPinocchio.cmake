#############################################################
#   Find Pinocchio (wrapper)
#   Written on 2025.12.23.
#############################################################

set(PACKAGE_NAME pinocchio)

get_filename_component(this_cmake_file ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
message(STATUS
    "\n-------------------- Finding ${PACKAGE_NAME} in ${this_cmake_file} --------------------------")

# Prefer config-mode
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG TRUE)

# Try to locate pinocchio via config package
find_package(pinocchio CONFIG REQUIRED)

if(pinocchio_FOUND)
    message(STATUS "pinocchio is found!")
    cmake_print_properties( TARGETS pinocchio::pinocchio
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
    message(FATAL_ERROR "pinocchio is not found but is required by the project.")
endif()

message(STATUS
    "-------------------- Finding ${PACKAGE_NAME} in ${this_cmake_file} --------------------------\n")
