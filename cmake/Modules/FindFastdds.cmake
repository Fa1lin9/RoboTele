#############################################################
#   Find fastdds (wrapper)
#   Written on 2025.12.25.
#############################################################

set(PACKAGE_NAME fastdds)

get_filename_component(this_cmake_file ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
message(STATUS
    "\n-------------------- Finding ${PACKAGE_NAME} in ${this_cmake_file} --------------------------")

# Prefer config-mode
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG TRUE)

# First, try to find FastDDS in /usr/local
find_package(${PACKAGE_NAME} CONFIG REQUIRED PATHS /usr/local)

# If not found, fall back to the default search paths
if(NOT ${PACKAGE_NAME}_FOUND)
    message(WARNING "${PACKAGE_NAME} not found in /usr/local, looking in default paths.")
    find_package(${PACKAGE_NAME} CONFIG REQUIRED)
endif()

if(${PACKAGE_NAME}_FOUND)
    message(STATUS "${PACKAGE_NAME} is found!")
    message(STATUS "${PACKAGE_NAME}_DIR is ${${PACKAGE_NAME}_DIR}")
    message(STATUS "${PACKAGE_NAME} library path: ${${PACKAGE_NAME}_LIBRARY}")
    message(STATUS "${PACKAGE_NAME} include path: ${${PACKAGE_NAME}_INCLUDE_DIRS}")
    message(STATUS "${PACKAGE_NAME} version: ${${PACKAGE_NAME}_VERSION}")
else()
    message(FATAL_ERROR "${PACKAGE_NAME} is not found but is required by the project.")
endif()

message(STATUS
    "-------------------- Finding ${PACKAGE_NAME} in ${this_cmake_file} --------------------------\n")
