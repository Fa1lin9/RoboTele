#############################################################
#	CMake requirement
#############################################################
cmake_minimum_required(VERSION 3.20)

set( msg_head "[Package Robot CMakeLists.txt] :" )

##########################################################################
#	MainRoutine group components definition
##########################################################################
set( GROUP_NAME "BasicSolver" )
list( APPEND ${GROUP_NAME}_Components
    HeadSolver
    WaistSolver
    
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
