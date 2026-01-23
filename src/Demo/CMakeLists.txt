#############################################################
#	CMake requirement
#############################################################
cmake_minimum_required(VERSION 3.20)

set( GROUP_NAME "Demo" )

set( msg_head "[Package ${GROUP_NAME} CMakeLists.txt] :" )

##########################################################################
#	MainRoutine group components definition
##########################################################################
list( APPEND ${GROUP_NAME}_Components
    # PybindDemo
    # SocketDemo
#    NLoptDemo
#    PinocchioDemo
    RobotDemo
    # CanDriverDemo
    MainDemo
#    FastDDSDemo
#    JsonDemo
    # ModBusDemo
    HandDemo
    # BasicSolverDemo
)

##########################################################################
#	MainParam group build definition
##########################################################################
foreach( each_comp_ ${${GROUP_NAME}_Components} )
    cmake_dependent_option( Build${GROUP_NAME}_${each_comp_} "${GROUP_NAME} : build ${each_comp_}" ON "BuildPackages_${GROUP_NAME}" OFF )
    if( ${Build${GROUP_NAME}_${each_comp_}} )
        add_subdirectory( ${each_comp_} )
    endif()
endforeach()
