############################################################
#   Pipe Arrangement
#   Written by Fa1lin9 on 2025.8.25
############################################################
cmake_minimum_required( VERSION 3.20 )
project( RobotTeleoperate
    VERSION 1.0.0
    LANGUAGES CXX
)

message( STATUS "Project Version: ${PROJECT_VERSION}" )
set( CMAKE_MODULE_PATH 
    ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/cmake 
)

#######################################################
#   Project Setting
#######################################################
include ( ProjectSetting )


#######################################################
#   CMake tools included
#######################################################
include( CMakeDependentOption )
include( CMakePrintHelpers )


############################################################
#	User Configuration
############################################################
include( UserConfiguration )


############################################################
#	Third Party Configuration
############################################################
#   Whether display the Boost target information
option( Display_Imported_Target_Boost "Whether display the properties of imported target boost." OFF )

#   Whether display the Qt target information
option( Display_Imported_Target_Qt5 "Whether display the properties of imported target Qt5." OFF )

#   Whether display the Qt target information
option( Display_Imported_Target "Whether display the properties of all imported targets." ON )


include( ThirdPartyConfiguration )



#######################################################
#      Define a file path for input/output            #
#######################################################
set( SOURCE_FILE_PATH  "${CMAKE_SOURCE_DIR}/" CACHE PATH "Project Source Path" FORCE )
configure_file( include/source_path.h.in include/source_path.h @ONLY)

#######################################################
#                     My Program                      #
#######################################################
option( BuildPackages_GUI "Whether generate GUI package?" ON )
if( BuildPackages_GUI )
    add_subdirectory( src/GUI )
endif()

option( BuildPackages_Demo "Whether generate Demo package?" ON )
if( BuildPackages_Demo )
    add_subdirectory( src/Demo )
endif()

option( BuildPackages_Core "Whether generate Core package?" ON )
if( BuildPackages_Core )
    add_subdirectory( src/Core )
endif()

option( BuildPackages_Teleoperate "Whether generate Core package?" ON )
if( BuildPackages_Teleoperate )
    add_subdirectory( src/Teleoperate )
endif()

option( BuildPackages_RobotHardware "Whether generate Core package?" ON )
if( BuildPackages_RobotHardware )
    add_subdirectory( src/RobotHardware )
endif()

option( BuildPackages_ArmSolver "Whether generate Core package?" ON )
if( BuildPackages_ArmSolver )
    add_subdirectory( src/ArmSolver )
endif()

option( BuildPackages_Transform "Whether generate Core package?" ON )
if( BuildPackages_Transform )
    add_subdirectory( src/Transform )
endif()

option( BuildPackages_FastDDSInterface "Whether generate Core package?" ON )
if( BuildPackages_FastDDSInterface )
    add_subdirectory( src/FastDDSInterface )
endif()

option( BuildPackages_BasicSolver "Whether generate Core package?" ON )
if( BuildPackages_BasicSolver )
    add_subdirectory( src/BasicSolver )
endif()

option( BuildPackages_HandController "Whether generate Core package?" ON )
if( BuildPackages_HandController )
    add_subdirectory( src/HandController )
endif()

option( BuildPackages_HandSolver "Whether generate Core package?" ON )
if( BuildPackages_HandSolver )
    add_subdirectory( src/HandSolver )
endif()

