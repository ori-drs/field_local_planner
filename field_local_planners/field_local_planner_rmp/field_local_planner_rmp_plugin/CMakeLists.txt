cmake_minimum_required(VERSION 3.0.2)
project(field_local_planner_rmp_plugin)
set (CMAKE_CXX_STANDARD 14)

set(DEFAULT_BUILD "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
   message(STATUS "Setting build type to '${DEFAULT_BUILD}' as none was specified.")
   set(CMAKE_BUILD_TYPE ${DEFAULT_BUILD} CACHE STRING "Choose the type of build." FORCE)
   # Set the possible values of build type for cmake-gui
   set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug"
                                                        "Release"
                                                        "MinSizeRel"
                                                        "RelWithDebInfo")
endif()

set(CATKIN_PACKAGE_LIST
  dynamic_reconfigure  
  grid_map_core
  grid_map_msgs
  field_local_planner_base_plugin
  field_local_planner_msgs
  field_local_planner_rmp
  pluginlib
  roscpp
  visualization_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_LIST}
)

# Libraries dependencies
# GTSAM
find_package(GTSAMCMakeTools REQUIRED)
set(GTSAM_BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "Enable/Disable building with all instructions supported by native architecture (binary may not be portable!)" FORCE)
find_package(GTSAM REQUIRED)

# Dynamic reconfigure
generate_dynamic_reconfigure_options(dynamic_reconfigure/Rmp.cfg)

# configure catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_PACKAGE_LIST}
)

###########
## Build ##
###########
include_directories(
  include
  SYSTEM
    ${GTSAM_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

# Declare library
add_library(${PROJECT_NAME}
            src/field_local_planner_rmp_plugin/rmp_plugin.cpp
)

target_include_directories(${PROJECT_NAME}
  SYSTEM PUBLIC
    ${GTSAM_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GTSAM_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS 
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  FILES
    field_local_planner_plugin.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
