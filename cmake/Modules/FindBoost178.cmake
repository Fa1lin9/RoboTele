#############################################################
#   Find Boost 1.78
#   Written by Fa1lin9 on 2025.12.23.
#############################################################

set(PACKAGE_NAME Boost178)

get_filename_component(this_cmake_file ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
message(STATUS
    "\n-------------------- Finding ${PACKAGE_NAME} in ${this_cmake_file} --------------------------")

# ------------------------------------------------------------
#   Prefer config mode
# ------------------------------------------------------------
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG TRUE)

# ------------------------------------------------------------
#   Boost components
# ------------------------------------------------------------
set(Boost_Components 
    locale date_time filesystem timer regex thread serialization system program_options json)

# ------------------------------------------------------------
#   Find Boost 1.78
# ------------------------------------------------------------
find_package(Boost 1.78 REQUIRED COMPONENTS ${Boost_Components})

# ------------------------------------------------------------
#   Print information if found
# ------------------------------------------------------------
if(Boost_FOUND)
    message(STATUS "Boost is found!")

    foreach(comp IN LISTS Boost_Components)
        message(STATUS "---------------------------------------------")
        message(STATUS "Component: ${comp} is found!")
        # cmake_print_properties(TARGETS Boost::${comp}
        #     PROPERTIES
        #         INCLUDE_DIRECTORIES
        #         INTERFACE_INCLUDE_DIRECTORIES
        #         IMPORTED_LOCATION_DEBUG
        #         IMPORTED_IMPLIB_DEBUG
        #         IMPORTED_LOCATION_RELEASE
        #         IMPORTED_IMPLIB_RELEASE
        #         IMPORTED_CONFIGURATIONS
        #         IMPORTED_LOCATION
        #         IMPORTED_IMPLIB
        # )
    endforeach()
else()
    message(FATAL_ERROR "Boost is not found! Required for your project!")
endif()

message(STATUS
    "-------------------- Finding ${PACKAGE_NAME} in ${this_cmake_file} --------------------------\n")
