############################################################
#   The dependant packages include
#   --  Qt 5.15.5
#   --  Boost 1.78.0
#   --  Eigen
#   --  pinocchio
#   --  nlopt
#   --  casadi
#   --  protobuf 21.12
#   --  libmodbus
#   Try to find each packages with an imported target
#   Written by Fa1lin9 in 2025.12.25
############################################################

list(APPEND CMAKE_PREFIX_PATH "/usr/local")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

# #######################################################
# #                     Find Boost 1.78                 #
# #           Boost should be found before pinocchio    #
# #######################################################

include( FindBoost178 )

# #######################################################
# #                     Find Qt                         #
# #######################################################

include( FindQt )

# #######################################################
# #                     Find libmodbus                  #
# #######################################################

include( FindLibmodbus )

# # #######################################################
# # #                     Find pinocchio                  #
# # #######################################################

# include( FindPinocchio )

# # #######################################################
# # #                     Find Eigen                      #
# # #######################################################

# include( FindEigen )

# # # #######################################################
# # # #                     Find jsoncons                   #
# # # #######################################################

# #  find_package( jsoncons REQUIRED )

#  # # #######################################################
#  # # #                     Find nlopt                      #
#  # # #######################################################

# include( FindNLopt )

# # # #######################################################
# # # #             Find Fastdds and Fastcdr                #
# # # #######################################################

# include( FindFastdds )
# include( FindFastcdr )

# # #######################################################
# # #                     Find protobuf                     #
# # #######################################################

# include( FindProtobuf )

# # # #######################################################
# # # #                     Find casadi                      #
# # # #######################################################

# include( FindCasadi )

# # # #######################################################
# # # #                     Find spdlog                    #
# # # #######################################################
# # find_package( spdlog REQUIRED)

# # # #######################################################
# # # #                     Find cppzmq                     #
# # # #######################################################

# find_package( cppzmq REQUIRED )

# #############################################################
#   Define the list of packages (only package names)
# #############################################################
set(PACKAGE_LIST 
    "pinocchio"
    "Eigen3"
    "NLopt"
    "Fastdds"
    "Fastcdr"
    "protobuf"
    "casadi"
    "cppzmq"

)

include( FindPackage )

# #############################################################
#   Loop through the package list and find each package
# #############################################################
foreach(PACKAGE_NAME IN LISTS PACKAGE_LIST)

    if(PACKAGE_NAME STREQUAL "casadi")
        # If it's casadi, set the version to 3.7.0
        FindPkgWrapper(${PACKAGE_NAME} "/usr/local" "3.7.0")
    else()
        # Call the FindPkgWrapper function for each package
        FindPkgWrapper(${PACKAGE_NAME})
    endif()

endforeach()

# # #######################################################
# # #            Find Ti5RobotSDK               #
# # #   Comes from the HumanoidDualArmSolver     #
# # #######################################################

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/ThirdPartyLibs/cmake")
# find_package(Ti5RobotSDK REQUIRED)
include( FindTi5RobotSDK )
