cmake_minimum_required(VERSION 3.0.2)
project(field_local_planners_ros)
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


# GTSAM
find_package(GTSAMCMakeTools REQUIRED)
set(GTSAM_BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "Enable/Disable building with all instructions supported by native architecture (binary may not be portable!)" FORCE)
include(GtsamBuildTypes)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  eigen_conversions
  geometry_msgs
  field_local_planners_plugins
  field_local_planners_msgs
  nav_msgs
  pluginlib
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  std_msgs
  tf_conversions
)

# Libraries dependencies
# configure catkin package
catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS  actionlib
                  actionlib_msgs
                  eigen_conversions
                  geometry_msgs
                  field_local_planners_plugins
                  field_local_planners_msgs
                  nav_msgs
                  pluginlib
                  pcl_conversions
                  pcl_ros
                  roscpp
                  std_msgs
                  std_msgs
                  tf_conversions
)

###########
## Build ##
###########
include_directories(
  include
  SYSTEM
    ${gtsam_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

# Declare node
add_executable(field_local_planners_node
               src/field_local_planners_ros/local_planner_handler.cpp
               src/local_planner_node.cpp)

target_link_libraries(field_local_planners_node
                      ${catkin_LIBRARIES}
                      gtsam
                      gtsam_unstable
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS 
    field_local_planners_node
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

############
## Python ##
############
catkin_install_python(PROGRAMS python/carrot_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)