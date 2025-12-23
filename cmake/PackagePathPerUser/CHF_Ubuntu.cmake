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

# 
list(APPEND CMAKE_PREFIX_PATH "/usr/local")

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

# # #######################################################
# # #              Find pybind and python                 #
# # #######################################################
# set(MY_CONDA_ENV "/home/fa1lin9/anaconda3/envs/tv")
# set(PYTHON_EXECUTABLE "${MY_CONDA_ENV}/bin/python")

# set(Python3_EXECUTABLE ${PYTHON_EXECUTABLE})
# find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# execute_process( COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir 
#             OUTPUT_VARIABLE pybind11_DIR 
#             OUTPUT_STRIP_TRAILING_WHITESPACE ) 

# find_package(pybind11 CONFIG REQUIRED PATHS ${pybind11_DIR})

# # find_package(pybind11 CONFIG REQUIRED
# # )

# message(STATUS "---------- pybind11 ----------")
# if (pybind11_FOUND)
#     message(STATUS "Found pybind11!")
#     message(STATUS "pybind11 version: ${pybind11_VERSION}")
#     message(STATUS "pybind11 include dirs: ${pybind11_INCLUDE_DIRS}")
    
#     message(STATUS ">>> Using Conda env: ${MY_CONDA_ENV}") 
#     message(STATUS ">>> Python exec: ${Python3_EXECUTABLE}") 
#     message(STATUS ">>> Python include: ${Python3_INCLUDE_DIRS}") 
#     message(STATUS ">>> Python lib: ${Python3_LIBRARIES}")
# else()
#     message(FATAL_ERROR "pybind11 not found!")
# endif()
# message(STATUS "---------- pybind11 ----------")

# #######################################################
# #                     Find Boost                      #
# #           Boost should be found before pinocchio    #
# #######################################################
set( CMAKE_FIND_PACKAGE_PREFER_CONFIG TRUE )

set(Boost_Components 
    locale date_time filesystem timer regex thread serialization system program_options json)

find_package(Boost 1.78 REQUIRED 
    COMPONENTS ${Boost_Components} 
)

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


# #######################################################
# #                     Find pinocchio                  #
# #######################################################
# in 404CAD do not have pinocchio

# set( coal_PATH /home/djr/ws_djr/djr_libs/hpp-fcl )
#    set( EIGENPY_PATH /opt/openrobots/lib/cmake/eigenpy )
#    set( PINOCCHIO_PATH /opt/openrobots/lib/cmake/pinocchio )
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

# # #######################################################
# # #            Find libRemoteAPIClient                  #
# # #######################################################

# set(RemoteAPIClient "/home/fa1lin9/CoppeliaSim/programming/zmqRemoteApi/clients/cpp/build" )
# find_library(RemoteAPIClient_LIB
#     NAMES RemoteAPIClient
#     PATHS ${RemoteAPIClient}
#     REQUIRED
# )

# # 查找头文件
# set( RemoteAPIClient_INCLUDE_DIR /home/fa1lin9/CoppeliaSim/programming/zmqRemoteApi/clients/cpp )

# # 打印查找结果
# message(STATUS "RemoteAPIClient_LIB Library: ${RemoteAPIClient_LIB}")
# message(STATUS "RemoteAPIClient_INCLUDE_DIR Include Directory: ${RemoteAPIClient_INCLUDE_DIR}")

# # #######################################################
# # #                     Find cppzmq                     #
# # #######################################################
 find_package( cppzmq REQUIRED )

# # #######################################################
# # #                     Find jsoncons                   #
# # #######################################################

#  find_package( jsoncons REQUIRED )

 # # #######################################################
 # # #                     Find nlopt                      #
 # # #######################################################

 find_package( NLopt REQUIRED )
 if( NLopt_FOUND )
    message( STATUS "NLopt is found!")
endif()

# # #######################################################
# # #                     Find Fastdds                      #
# # #######################################################

find_package( fastdds REQUIRED )
find_package( fastcdr REQUIRED )

# #######################################################
# #                     Find protobuf                     #
# #######################################################

find_package( protobuf REQUIRED )

# # #######################################################
# # #                     Find CRP_SDK                    #
# # #######################################################

# set(CRP_SDK "/home/fa1lin9/ProgramEnv/CrobotpOSSDK" )
# set(CRP_SDK_LIB_PATH "/home/fa1lin9/ProgramEnv/CrobotpOSSDK/bin")
# set(CRP_SDK_HEAD_PATH "/home/fa1lin9/ProgramEnv/CrobotpOSSDK/cpp/include")
# find_library(CRP_LIBS
#     NAMES RobotService
#     PATHS ${CRP_SDK_LIB_PATH}
#     REQUIRED
# )

# # 查找头文件
# set( CRP_INCLUDE_DIR ${CRP_SDK_HEAD_PATH} )

# # 打印查找结果
# message(STATUS "CRP_LIB Library: ${CRP_LIBS}")
# message(STATUS "CRP_HEAD Include Directory: ${CRP_INCLUDE_DIR}")

# # #######################################################
# # #                     Find spdlog                    #
# # #######################################################
# find_package( spdlog REQUIRED)

# # #######################################################
# # #            Find Ti5RobotCtrl               #
# # #   Comes from the HumanoidDualArmSolver     #
# # #######################################################

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/ThirdPartyLibs/cmake")
find_package(Ti5RobotCtrl REQUIRED)
