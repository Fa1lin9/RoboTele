#############################################################
#   Find Ti5RobotCtrl
#   Written by Fa1lin9 on 2025.12.22.
#############################################################

set(PACKAGE_NAME Ti5RobotCtrl)

get_filename_component(this_cmake_file ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
message(STATUS
    "\n-------------------- Finding ${PACKAGE_NAME} in ${this_cmake_file} --------------------------")

# ------------------------------------------------------------
#   Detect Platform and Architecture
# ------------------------------------------------------------
include(${CMAKE_CURRENT_LIST_DIR}/DetectPlatform.cmake)

# ------------------------------------------------------------
#   ThirdParty root
# ------------------------------------------------------------
get_filename_component(CURRENT_DIR ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
get_filename_component(ThirdPartyLibs_ROOT ${CURRENT_DIR}/.. ABSOLUTE)

# ------------------------------------------------------------
#   Platform/Arch specific library root
# ------------------------------------------------------------
set(PLATFORM_LIB_ROOT
    ${ThirdPartyLibs_ROOT}/${CURRENT_OS}/${CURRENT_ARCH}
    CACHE PATH "Platform and architecture specific ThirdPartyLibs root")

# ------------------------------------------------------------
#   Package root
# ------------------------------------------------------------
set(Ti5RobotCtrl_ROOT
    ${PLATFORM_LIB_ROOT}/${PACKAGE_NAME}
    CACHE PATH "Root path of ${PACKAGE_NAME}")

set(Ti5RobotCtrl_INCLUDE_DIR
    ${Ti5RobotCtrl_ROOT}/include)

# ------------------------------------------------------------
#   Library search path 
# ------------------------------------------------------------
set(Ti5RobotCtrl_LIB_DIR ${Ti5RobotCtrl_ROOT}/lib)

# ------------------------------------------------------------
#   Create imported target
# ------------------------------------------------------------
add_library(Ti5RobotCtrl INTERFACE)

file(GLOB ALL_LIBS "${Ti5RobotCtrl_LIB_DIR}/*.so" "${Ti5RobotCtrl_LIB_DIR}/*.a")

foreach(lib_path IN LISTS ALL_LIBS)
    get_filename_component(lib_name ${lib_path} NAME_WE)

    if(lib_path MATCHES "\\.a$|\\.lib$")
        set(lib_type STATIC)
    elseif(lib_path MATCHES "\\.so$|\\.dll$")
        set(lib_type SHARED)
    else()
        message(FATAL_ERROR "Unsupported library type: ${lib_path}")
    endif()

    add_library(${lib_name} ${lib_type} IMPORTED)
    set_target_properties(${lib_name} PROPERTIES
        IMPORTED_LOCATION ${lib_path}
    )

    target_link_libraries(Ti5RobotCtrl INTERFACE ${lib_name})
endforeach()

target_include_directories(Ti5RobotCtrl INTERFACE ${Ti5RobotCtrl_INCLUDE_DIR})

add_library(Ti5RobotCtrl::Ti5RobotCtrl ALIAS Ti5RobotCtrl)

message(STATUS "Ti5RobotCtrl_INCLUDE_DIR = ${Ti5RobotCtrl_INCLUDE_DIR}")
message(STATUS "Ti5RobotCtrl libraries = ${ALL_LIBS}")


message(STATUS
    "-------------------- Finding ${PACKAGE_NAME} in ${this_cmake_file} --------------------------\n")
