#############################################################
#   Find Package Wrapper (Generic)
#   Written on 2025.12.25.
#############################################################

# Define the find_package wrapper function
function(FindPkgWrapper PACKAGE_NAME)

    message(STATUS "\n-------------------- Finding ${PACKAGE_NAME} in ${this_cmake_file} --------------------------")

    # Set default values for SEARCH_PATH and PACKAGE_VERSION
    if(NOT DEFINED SEARCH_PATH)
        set(SEARCH_PATH "/usr/local")  # Default search path
    endif()

    if(NOT DEFINED PACKAGE_VERSION)
        set(PACKAGE_VERSION "")  # Default empty version
    endif()

    # Check if SEARCH_PATH is passed as a second argument
    if(ARGC GREATER 1)
        list(GET ARGV 1 SEARCH_PATH)  # If the second argument exists, use it as SEARCH_PATH
        message(STATUS "Specified SEARCH_PATH: ${SEARCH_PATH}")
    endif()

    # Check if PACKAGE_VERSION is passed as a third argument
    if(ARGC GREATER 2)
        list(GET ARGV 2 PACKAGE_VERSION)  # If the third argument exists, use it as PACKAGE_VERSION
        message(STATUS "Specified PACKAGE_VERSION: ${PACKAGE_VERSION}")
    endif()

    # Get the directory of the current CMakeLists.txt file
    get_filename_component(this_cmake_file ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)

    # Prefer config-mode for finding the package
    set(CMAKE_FIND_PACKAGE_PREFER_CONFIG TRUE)

    # If SEARCH_PATH is specified and exists, search in that path
    if(EXISTS ${SEARCH_PATH})
        message(STATUS "Searching for ${PACKAGE_NAME} in ${SEARCH_PATH}")
        find_package(${PACKAGE_NAME} CONFIG REQUIRED PATHS ${SEARCH_PATH})
    else()
        message(STATUS "No custom SEARCH_PATH specified or the path doesn't exist. Searching in default paths.")
        find_package(${PACKAGE_NAME} CONFIG REQUIRED)
    endif()

    # If the package is found, output the package details
    if(${PACKAGE_NAME}_FOUND)
        message(STATUS "${PACKAGE_NAME} is found!")
        message(STATUS "${PACKAGE_NAME}_DIR is ${${PACKAGE_NAME}_DIR}")
        message(STATUS "${PACKAGE_NAME} library path: ${${PACKAGE_NAME}_LIBRARY}")
        message(STATUS "${PACKAGE_NAME} include path: ${${PACKAGE_NAME}_INCLUDE_DIRS}")
        message(STATUS "${PACKAGE_NAME} version: ${${PACKAGE_NAME}_VERSION}")
    else()
        message(FATAL_ERROR "${PACKAGE_NAME} is not found but is required by the project.")
    endif()

    message(STATUS "-------------------- Finding ${PACKAGE_NAME} in ${this_cmake_file} --------------------------\n")
endfunction()

