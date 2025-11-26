############################################################
#   Find Cutting For Welding dependant packages for user tangtang 4090 Windows

#   The dependant packages include
#   --  Qt 5.15.5
#   --  Boost 1.78.0
#   --  Eigen
#   --  libfranka
#   --  pinocchio
#   --  Python3
#   --  nlopt
#   Try to find each packages with an imported target
#   Written by djr in 2025.5
############################################################

# #######################################################
# #                     Find Eigen                      #
# #######################################################

find_package(Eigen3 REQUIRED)

if (Eigen3_FOUND)
        message( STATUS "Eigen3 is found!" )
#        get_target_property(Eigen_DIR Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
#        message( STATUS "Eigen_DIR is ${Eigen_DIR}. ")
        cmake_print_properties( TARGETS Eigen3::Eigen
            PROPERTIES
                INCLUDE_DIRECTORIES
                INTERFACE_INCLUDE_DIRECTORIES
                IMPORTED_LOCATION_DEBUG
                IMPORTED_IMPLIB_DEBUG
                IMPORTED_LOCATION_RELEASE
                IMPORTED_IMPLIB_RELEASE
                IMPORTED_CONFIGURATIONS
                IMPORTED_LOCATION
                IMPORTED_IMPLIB
                )
endif()


# #######################################################
# #                     Find Boost                      #
# #           Boost should be found before pinocchio    #
# #######################################################
set( CMAKE_FIND_PACKAGE_PREFER_CONFIG TRUE )

set(Boost_Components locale date_time filesystem timer regex thread serialization system program_options json)
find_package(Boost 1.78 REQUIRED 
    COMPONENTS ${Boost_Components} 
    PATHS /usr/local/lib/cmake/Boost-1.78.0)

if(Boost_FOUND)
    message( STATUS "Boost is found!")
    cmake_print_properties( TARGETS Boost::headers
        PROPERTIES
            INCLUDE_DIRECTORIES
            INTERFACE_INCLUDE_DIRECTORIES
            IMPORTED_LOCATION_DEBUG
            IMPORTED_IMPLIB_DEBUG
            IMPORTED_LOCATION_RELEASE
            IMPORTED_IMPLIB_RELEASE
            IMPORTED_CONFIGURATIONS
            IMPORTED_LOCATION
            IMPORTED_IMPLIB
            )

    cmake_print_properties( TARGETS Boost::serialization
        PROPERTIES
            INCLUDE_DIRECTORIES
            INTERFACE_INCLUDE_DIRECTORIES
            IMPORTED_LOCATION_DEBUG
            IMPORTED_IMPLIB_DEBUG
            IMPORTED_LOCATION_RELEASE
            IMPORTED_IMPLIB_RELEASE
            IMPORTED_CONFIGURATIONS
            IMPORTED_LOCATION
            IMPORTED_IMPLIB
            )
else()
    message(FATAL_ERROR "Boost is not found! Timing is not disabled! ")
endif()
## #######################################################
## #                     Find urdfdom                    #
## #######################################################
find_package( urdfdom CONFIG REQUIRED )
if(urdfdom_FOUND)
    message(STATUS "urdfdom is FOUND with version = ${urdfdom_Version}!")

    #urdfdom_sensor;urdfdom_model_state;urdfdom_model;urdfdom_world

    if( TARGET urdfdom::urdfdom_sensor )
        message( STATUS "Target urdfdom::urdfdom_sensor is found!")
    endif()

    if( TARGET urdfdom::urdf_parser )
        message( STATUS "Target urdfdom::urdf_parser is found!")
    else()
        message( WARNING "Target urdfdom::urdf_parser is NOT found!")
        add_library(urdf_parser INTERFACE)
#            target_include_directories(urdf_parser INTERFACE
#              "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#              "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
        target_link_libraries(urdf_parser INTERFACE
          urdfdom::urdfdom_model
          urdfdom::urdfdom_sensor
          urdfdom::urdfdom_world)
        add_library(urdfdom::urdf_parser ALIAS urdf_parser)
    endif()
else()
endif()

message(STATUS "URDFDOM_FOUND: ${URDFDOM_FOUND}")
message(STATUS "URDFDOM_INCLUDE_DIRS: ${URDFDOM_INCLUDE_DIRS}")
message(STATUS "URDFDOM_LIBRARIES: ${URDFDOM_LIBRARIES}")

# #######################################################
# #                     Find pinocchio                  #
# #######################################################
# in 404CAD do not have pinocchio

find_package( eigenpy CONFIG REQUIRED )
#    find_package( hpp-fcl REQUIRED )
find_package( pinocchio CONFIG REQUIRED )

if( pinocchio_FOUND )
    message( STATUS "pinocchio is found!")
endif()


# #######################################################
# #                     Find Threads                    #
# #######################################################
# find_package(Threads REQUIRED)
# if (Threads_FOUND)
#         message(STATUS "Threads is found!")
# endif()


# #######################################################
# #                     Find Qt                         #
# #######################################################


set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
find_package( Qt5 REQUIRED COMPONENTS Core Widgets OpenGL Charts )
#find_package(Qt5 REQUIRED COMPONENTS Widgets UiTools)
if (Qt5_FOUND)
    message(STATUS "Qt 5 found!")
else()
    message(FATAL_ERROR "Qt 5 not found!")
endif()



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
# # #                     Find Ros                      #
# # #######################################################

# set(CMAKE_PREFIX_PATH "/opt/ros/humble" ${CMAKE_PREFIX_PATH})

##find_package(ament_cmake REQUIRED)
#find_package( rclcpp REQUIRED )
# find_package( std_msgs REQUIRED )

# # #######################################################
# # #                     Find casadi                      #
# # #######################################################

find_package( casadi REQUIRED )

 if (Qt5_FOUND)
    message(STATUS "casadi found!")
else()
    message(FATAL_ERROR "casadi not found!")
endif()

# # #######################################################
# # #            Find HumanoidDualArmSolver               #
# # #######################################################
#set(HumaniodRobot "/home/djr/djr_workspace/djr_libs/HumanoidDualArmSolver" )
set(HumaniodRobot "/home/ti5robot/workspace_djr/djr_lib/HumanoidDualArmSolver" )
set(HumaniodRobot_include "${HumaniodRobot}/include")

find_library(MYLIBTI5_LIB
    NAMES mylibti5_arm_2004
    PATHS "${HumaniodRobot}/usrlib/arm/2004"
    REQUIRED
)

# message(STATUS "CONTROLCAN_LIB Library: ${CONTROLCAN_LIB}")

# find_library(HumaniodRobot_CONTROLCAN_LIB
#     NAMES controlcan
#     PATHS "${HumaniodRobot}/usrlib/arm/2004"
#     REQUIRED
# )

find_library(CONTROLCAN_LIB
    NAMES controlcan
    PATHS "${HumaniodRobot}/usrlib/arm/2004"
    REQUIRED
)

set(HumaniodRobot_LIBS
    ${MYLIBTI5_LIB}
    ${CONTROLCAN_LIB}
    # ${HumaniodRobot_CONTROLCAN_LIB}
)

# 打印查找结果
message(STATUS "HumaniodRobot_include: ${HumaniodRobot_include}")
message(STATUS "MYLIBTI5_LIB Library: ${MYLIBTI5_LIB}")
message(STATUS "CONTROLCAN_LIB Library: ${CONTROLCAN_LIB}")
# message(STATUS "HumaniodRobot_CONTROLCAN_LIB Library: ${HumaniodRobot_CONTROLCAN_LIB}")
# message(STATUS "HumaniodRobot_LIBS Library: ${HumaniodRobot_LIBS}")

# #######################################################
# #                     Find libmodbus                  #
# #######################################################

# 查找头文件
find_path(libmodbus_INCLUDE_DIR
    NAMES modbus/modbus.h
    PATHS /usr/local/include
)

# 查找库文件
find_library(libmodbus_LIBRARY
    NAMES modbus
    PATHS /usr/local/lib
)
