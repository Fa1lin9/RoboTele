#############################################################
#   Find libmodbus
#   Written by Fa1lin9 on 2025.12.22.
#############################################################

set(PACKAGE_NAME libmodbus)

get_filename_component(this_cmake_file ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
message(STATUS
    "\n-------------------- Finding ${PACKAGE_NAME} in ${this_cmake_file} --------------------------")

# include
find_path(libmodbus_INCLUDE_DIR
    NAMES modbus/modbus.h
    PATHS /usr/local/include /usr/include
)

# lib
find_library(libmodbus_LIBRARY
    NAMES modbus
    PATHS /usr/local/lib /usr/lib
)

if(libmodbus_INCLUDE_DIR AND libmodbus_LIBRARY)
    message(STATUS "libmodbus found:")
    message(STATUS "  include = ${libmodbus_INCLUDE_DIR}")
    message(STATUS "  library = ${libmodbus_LIBRARY}")

    add_library(libmodbus INTERFACE)

    target_include_directories(libmodbus INTERFACE
        ${libmodbus_INCLUDE_DIR}
    )

    target_link_libraries(libmodbus INTERFACE
        ${libmodbus_LIBRARY}
    )

    add_library(libmodbus::libmodbus ALIAS libmodbus)
else()
    message(FATAL_ERROR "libmodbus not found")
endif()


message(STATUS
    "-------------------- Finding ${PACKAGE_NAME} in ${this_cmake_file} --------------------------\n")