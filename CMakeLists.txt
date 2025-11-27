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
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/cmake )

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
include( MyInstallFuncDef )


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
    add_subdirectory( GUI )
endif()

option( BuildPackages_Demo "Whether generate Demo package?" ON )
if( BuildPackages_Demo )
    add_subdirectory( Demo )
endif()

option( BuildPackages_Core "Whether generate Core package?" ON )
if( BuildPackages_Core )
    add_subdirectory( Core )
endif()

option( BuildPackages_Teleoperate "Whether generate Core package?" ON )
if( BuildPackages_Teleoperate )
    add_subdirectory( Teleoperate )
endif()

option( BuildPackages_PhysicalRobot "Whether generate Core package?" ON )
if( BuildPackages_PhysicalRobot )
    add_subdirectory( PhysicalRobot )
endif()

option( BuildPackages_SimRobot "Whether generate Core package?" ON )
if( BuildPackages_SimRobot)
    add_subdirectory( SimRobot )
endif()

option( BuildPackages_ArmSolver "Whether generate Core package?" ON )
if( BuildPackages_ArmSolver )
    add_subdirectory( ArmSolver )
endif()

option( BuildPackages_CoordinateTransform "Whether generate Core package?" ON )
if( BuildPackages_CoordinateTransform )
    add_subdirectory( CoordinateTransform )
endif()

option( BuildPackages_FastDDSInterface "Whether generate Core package?" ON )
if( BuildPackages_FastDDSInterface )
    add_subdirectory( FastDDSInterface )
endif()

option( BuildPackages_BasicSolver "Whether generate Core package?" ON )
if( BuildPackages_BasicSolver )
    add_subdirectory( BasicSolver )
endif()

option( BuildPackages_HandController "Whether generate Core package?" ON )
if( BuildPackages_HandController )
    add_subdirectory( HandController )
endif()

option( BuildPackages_HandSolver "Whether generate Core package?" ON )
if( BuildPackages_HandSolver )
    add_subdirectory( HandSolver )
endif()

