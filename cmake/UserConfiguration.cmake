#######################################################
#   User Configuration
#   Make sure only one user is defined in the program
#######################################################

#   Choose User
option( USER_CHF_Ubuntu "Configure CHF Ubuntu system." OFF )
option( USER_Ti5Robot_Ubuntu "Configure Ti5Robot Ubuntu system." OFF )
option( USER_PK_Ubuntu "Configure PK Ubuntu system." OFF )
option( USER_Default "Configure Default system." ON )

#   Make Users as a group
list( APPEND USER_GROUP
    USER_CHF_Ubuntu
    USER_Ti5Robot_Ubuntu
    USER_PK_Ubuntu
    USER_Default
)

#   Get how many users are selected
set( NUMBER_OF_USER 0 )
foreach( each_user ${USER_GROUP} )
    if( ${each_user} )
        math( EXPR NUMBER_OF_USER "${NUMBER_OF_USER} + 1" )
        message( STATUS "Current User is ${each_user}. " )
    endif()
endforeach()

#   Return a fatal error if the selected user is not equal to 1
if( NOT ${NUMBER_OF_USER} EQUAL 1 )
    message( FATAL_ERROR "The number of user defined is ${NUMBER_OF_USER}, not equal to 1." )
endif()


