cmake_minimum_required(VERSION 3.0.2)
project(local_planners_drs_ros)
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


find_package(catkin REQUIRED COMPONENTS
  grid_map_core
  grid_map_msgs
  local_planners_drs
)

# Libraries dependencies
# YAML-cpp
find_package(yaml-cpp REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# GTSAM
find_package(GTSAMCMakeTools REQUIRED)
set(GTSAM_BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "Enable/Disable building with all instructions supported by native architecture (binary may not be portable!)" FORCE)
include(GtsamBuildTypes)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)

# PCL
set(PCL_FIND_QUIETLY ON)
find_package(PCL REQUIRED
  COMPONENTS
  common
  filters
  io
  kdtree
  sample_consensus
  segmentation
  surface
  visualization
)

# configure catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS  grid_map_core
                  grid_map_msgs
                  local_planners_drs
  DEPENDS OpenCV
          PCL
          YAML_CPP
)

###########
## Build ##
###########
include_directories(
  include
  ${gtsam_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Declare planners library
add_library(${PROJECT_NAME} 
                src/local_planners_drs_ros/planner_falco_plugin.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  gtsam
  gtsam_unstable
  ${YAML_CPP_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  FILES local_planners_drs_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
