#############################################################
#   Find Qt (wrapper for Qt5)
#   Written on 2025.12.23.
#############################################################

set(PACKAGE_NAME Qt)

get_filename_component(this_cmake_file ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
message(STATUS
    "\n-------------------- Finding ${PACKAGE_NAME} in ${this_cmake_file} --------------------------")

# Prefer config-mode
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG TRUE)

# Enable automoc/autouic/autorcc by default for convenience
set(CMAKE_AUTOUIC ON CACHE BOOL "Automatically run uic on .ui files" FORCE)
set(CMAKE_AUTOMOC ON CACHE BOOL "Automatically run moc on files" FORCE)
set(CMAKE_AUTORCC ON CACHE BOOL "Automatically run rcc on .qrc files" FORCE)

# Requested default Qt components
set(Qt5_components Core Widgets OpenGL Charts)

# Try to locate Qt5
find_package(Qt5 REQUIRED COMPONENTS ${Qt5_components})

if (Qt5_FOUND)
    message(STATUS "Qt5 is found!")

    foreach(comp IN LISTS Qt5_components)
        set(temp Qt5::${comp})
        if(TARGET ${temp})
            message(STATUS "---------------------------------------------")
            message(STATUS "Qt5 component: ${comp} is found!")
            # cmake_print_properties(TARGETS ${temp}
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
        endif()
    endforeach()

else()
    message(FATAL_ERROR "Qt5 is not found but is required by the project.")
endif()

message(STATUS
    "-------------------- Finding ${PACKAGE_NAME} in ${this_cmake_file} --------------------------\n")
