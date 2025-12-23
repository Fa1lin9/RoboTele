############################################################
#   Find Cutting For Welding dependant packages for user tangtang 4090 Windows

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
#   Written by Fa1lin9 in 2025.12.23
############################################################

list(APPEND CMAKE_PREFIX_PATH "/usr/local")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

# #######################################################
# #                     Find Eigen                      #
# #######################################################

include( FindEigen )

# #######################################################
# #                     Find Boost 1.78                 #
# #           Boost should be found before pinocchio    #
# #######################################################

include( FindBoost178 )

# #######################################################
# #                     Find pinocchio                  #
# #######################################################

include( FindPinocchio )

# #######################################################
# #                     Find Qt                         #
# #######################################################

include( FindQt )

# #######################################################
# #                     Find ZeroMQ                     #
# #######################################################

find_package( ZeroMQ REQUIRED )

# #######################################################
# #                     Find protobuf                     #
# #######################################################

find_package( protobuf REQUIRED )

# # #######################################################
# # #                     Find cppzmq                     #
# # #######################################################
find_package( cppzmq REQUIRED )

# # #######################################################
# # #                     Find nlopt                      #
# # #######################################################

find_package( NLopt REQUIRED )

# # #######################################################
# # #                     Find Fastdds                      #
# # #######################################################

find_package( fastdds REQUIRED )
find_package( fastcdr REQUIRED )

# # #######################################################
# # #                     Find casadi                      #
# # #######################################################

find_package( casadi REQUIRED )

 if (casadi_FOUND)
    message(STATUS "casadi found!")
else()
    message(FATAL_ERROR "casadi not found!")
endif()

# #######################################################
# #                     Find libmodbus                  #
# #######################################################

include( FindLibmodbus )

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/ThirdPartyLibs/cmake")
# find_package(Ti5RobotSDK REQUIRED)
include( FindTi5RobotSDK )
