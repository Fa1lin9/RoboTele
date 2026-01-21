#############################################################
#	CMake requirement
#############################################################
cmake_minimum_required( VERSION 3.20 )

#set(CMAKE_POSITION_INDEPENDENT_CODE ON)
#############################################################
#	Set project name, files, dependancies
#############################################################
set( TARGET_NAME "Demo" )
message( STATUS "\n-------------  ${GROUP_NAME} - ${TARGET_NAME}  -------------" )


#############################################################
#	Add the head and source files to the current project
#############################################################
# aux_source_directory( ${CMAKE_CURRENT_SOURCE_DIR} SOURCES )

file(GLOB SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cxx"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.ipp"
)

file( GLOB HEADERS ./*.h  *.hpp )
file( GLOB INTERFACE_HEADERS 
    ${PROJECT_SOURCE_DIR}/include/${GROUP_NAME}/${TARGET_NAME}/*.h
    ${PROJECT_SOURCE_DIR}/include/${GROUP_NAME}/${TARGET_NAME}/*.hpp
)


#############################################################
#	Create target and Set the target prorperties
#############################################################
#	Create target
add_library( ${TARGET_NAME} ${SOURCES} ${HEADERS} ${INTERFACE_HEADERS} )
add_library( ${GROUP_NAME}::${TARGET_NAME} ALIAS ${TARGET_NAME})

#  Set project include directories
target_compile_definitions( ${TARGET_NAME} PUBLIC -DSIM_REMOTEAPICLIENT_OBJECTS )
target_include_directories( ${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/${GROUP_NAME}>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/common>
    PRIVATE
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
# target_include_directories( ${TARGET_NAME} PUBLIC "." ${CMAKE_BINARY_DIR}/include ${CMAKE_SOURCE_DIR}/${TARGET_NAME} )


#   Add dependancies
target_link_libraries( ${TARGET_NAME}
    PUBLIC 

    PRIVATE
        $<BUILD_INTERFACE:fastdds>
        $<BUILD_INTERFACE:fastcdr>
)

# find_package( ament_cmake REQUIRED )
# ament_target_dependencies(${TARGET_NAME}
#     std_msgs
#     fastdds
#     fastcdr
# )

# #   Add Macro compile defination
# TARGET_COMPILE_DEFINITIONS( ${TARGET_NAME}
#     #PRIVATE
#     PUBLIC
#         -DNOMINMAX
# )

#   Set project properties
set_target_properties( ${TARGET_NAME} PROPERTIES
    FOLDER                  ${GROUP_NAME}
    DEBUG_POSTFIX           _static${CMAKE_DEBUG_POSTFIX}
    RELEASE_POSTFIX         _static${CMAKE_RELEASE_POSTFIX}
)
# #  Set project compile definitions
# target_compile_definitions( ${TARGET_NAME} 
#     # PUBLIC
#     PRIVATE
#         GACUT_SHAREDLIB_EXPORTS
# )

message( STATUS "-------------  ${GROUP_NAME} - ${TARGET_NAME}  -------------\n" )

