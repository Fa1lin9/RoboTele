#############################################################
#	CMake requirement
#############################################################
cmake_minimum_required( VERSION 3.20 )


#############################################################
#	Set project name, files, dependancies
#############################################################
set( TARGET_NAME "SimpleGUI" )
message( STATUS "\n-------------  ${GROUP_NAME} - ${TARGET_NAME}  -------------" )

#############################################################
#	Qt Setting
#############################################################
#   Qt Setting
set( CMAKE_AUTOUIC ON )
set( CMAKE_AUTOMOC ON )
set( CMAKE_AUTORCC ON )

#   Handle ui file
file( GLOB UI_FILES *.ui )

#   通过Ui文件生成对应的头文件，一定要添加
option( Using_Qt5 "Whether use Qt5?" ON )
if( Using_Qt5 )
    qt5_wrap_ui( WRAP_FILES ${UI_FILES} )
else()
    qt6_wrap_ui( WRAP_FILES ${UI_FILES} )
endif()

#将ui文件和生成文件整理在一个文件夹中，非必须
source_group( "Form Files" FILES ${UI_FILES} ${WRAP_FILES} )

#添加资源文件，非必须，一旦采用，注意修改相应的qrc文件名
file( GLOB RCC_FILES *.qrc )


#############################################################
#	Add the head and source files to the current project
#############################################################
aux_source_directory( ${CMAKE_CURRENT_SOURCE_DIR} SRC_FILES )

file( GLOB HEADER_FILES *.h  *.hpp )


#############################################################
#	Create target and Set the target prorperties
#############################################################
#	Create target
add_executable( ${TARGET_NAME} ${SRC_FILES} ${HEADER_FILES} ${WRAP_FILES} ${RCC_FILES} )

#  Set project include directories
target_include_directories( ${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/${GROUP_NAME}>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/common>
    PRIVATE
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# target_include_directories( ${TARGET_NAME} PUBLIC "." ${CMAKE_BINARY_DIR}/include ${CMAKE_SOURCE_DIR}/${TARGET_NAME} )


#   Set the depedant libraries
if( Using_Qt5 )
    message( STATUS "${TARGET_NAME} links Qt5!" )
    target_link_libraries( ${TARGET_NAME}
        PRIVATE
            $<BUILD_INTERFACE:Qt5::Widgets>
            $<BUILD_INTERFACE:Qt5::Charts>
            
            $<BUILD_INTERFACE:Boost::headers>
            $<BUILD_INTERFACE:Boost::timer>
            $<BUILD_INTERFACE:Boost::filesystem>
            $<BUILD_INTERFACE:Boost::thread>

            
            
    )
else()
    message( STATUS "${TARGET_NAME} links Qt6!" )
    target_link_libraries( ${TARGET_NAME}
        PRIVATE
            $<BUILD_INTERFACE:Qt6::Widgets>
            $<BUILD_INTERFACE:Qt6::Charts>
            
            $<BUILD_INTERFACE:Boost::headers>
            $<BUILD_INTERFACE:Boost::timer>
            $<BUILD_INTERFACE:Boost::filesystem>
            $<BUILD_INTERFACE:Boost::thread>
            
            $<BUILD_INTERFACE:CRPRobotMainGUI::BasicControlWidget>
    )
endif()

# #   Add Macro compile defination
# TARGET_COMPILE_DEFINITIONS( ${TARGET_NAME}
#     #PRIVATE
#     PUBLIC
#         -DNOMINMAX
# )

#   Set project properties
set_target_properties( ${TARGET_NAME} PROPERTIES
    FOLDER                  ${GROUP_NAME}
    DEBUG_POSTFIX           ${CMAKE_DEBUG_POSTFIX}
    RELEASE_POSTFIX         ${CMAKE_RELEASE_POSTFIX}
)


#############################################################
#	Set vs debug environment
#############################################################
string( JOIN ";" VSDebugEnvPath ${VSDebugEnvPath} ${Qt5_RUNTIME_DIR} ${GLEW_RUNTIME_DIR} ${Qt6_RUNTIME_DIR} )
message( STATUS "VSDebugEnvPath in TestNest_GUI is ${VSDebugEnvPath}. " )
if( CMAKE_SYSTEM_NAME MATCHES "Windows" )
    set_property( TARGET ${TARGET_NAME} PROPERTY 
        VS_DEBUGGER_ENVIRONMENT $<$<CONFIG:Debug>:${VSDebugEnvPath}> )    
endif()

message( STATUS "-------------  ${GROUP_NAME} - ${TARGET_NAME}  -------------\n" )
