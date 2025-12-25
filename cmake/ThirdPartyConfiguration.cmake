#   Package path config

# USER_CHF_Ubuntu
if( ${USER_CHF_Ubuntu} )
    include( PackagePathPerUser/CHF_Ubuntu )
endif()

if( ${USER_CrpRobot_Ubuntu} )
    include( PackagePathPerUser/Ti5Robot_Ubuntu )
endif()

if( ${USER_PK_Ubuntu} )
    include( PackagePathPerUser/PK_Ubuntu )
endif()