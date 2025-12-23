#############################################################
#   Find Ti5RobotSDK
#   Written by Fa1lin9 on 2025.12.22.
#############################################################

set(PACKAGE_NAME Ti5RobotSDK)

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
set(Ti5RobotSDK_ROOT
    ${PLATFORM_LIB_ROOT}/${PACKAGE_NAME}
    CACHE PATH "Root path of ${PACKAGE_NAME}")

set(Ti5RobotSDK_INCLUDE_DIR
    ${Ti5RobotSDK_ROOT}/include)

# ------------------------------------------------------------
#   Library search path 
# ------------------------------------------------------------
set(Ti5RobotSDK_LIB_DIR ${Ti5RobotSDK_ROOT}/lib)

# ------------------------------------------------------------
#   Library Construction 
# ------------------------------------------------------------
if("${CURRENT_ARCH}" STREQUAL "aarch64")
    set(MYLIBTI5_NAME "mylibti5_arm_2004")
else()

    set(MYLIBTI5_NAME "mylibti5_2004")
endif()

find_library(MYLIBTI5_LIB
    NAMES ${MYLIBTI5_NAME}
    PATHS ${Ti5RobotSDK_LIB_DIR}
    NO_DEFAULT_PATH
    REQUIRED
)

find_library(CONTROLCAN_LIB
    NAMES controlcan
    PATHS ${Ti5RobotSDK_LIB_DIR}
    NO_DEFAULT_PATH
    REQUIRED
)

set(Ti5RobotSDK_LIBS
    ${MYLIBTI5_LIB}
    ${CONTROLCAN_LIB}
)

if(NOT TARGET Ti5RobotSDK)
    add_library(Ti5RobotSDK INTERFACE)
    
    # include
    target_include_directories(Ti5RobotSDK INTERFACE ${Ti5RobotSDK_INCLUDE_DIR})
    
    # libs
    target_link_libraries(Ti5RobotSDK INTERFACE ${Ti5RobotSDK_LIBS})  

    add_library(Ti5RobotSDK::Ti5RobotSDK ALIAS Ti5RobotSDK)
endif()


message(STATUS "Ti5RobotSDK_INCLUDE_DIR = ${Ti5RobotSDK_INCLUDE_DIR}")
message(STATUS "Ti5RobotSDK_LIBS = ${Ti5RobotSDK_LIBS}")


message(STATUS
    "-------------------- Finding ${PACKAGE_NAME} in ${this_cmake_file} --------------------------\n")
