cmake_minimum_required(VERSION 3.0.2)
project(locally_reactive_controller)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  roscpp
  rospy
  pcl_conversions
  pcl_ros
  tf_conversions
  dynamic_reconfigure
  actionlib
  actionlib_msgs
  message_generation
  geometry_msgs
  grid_map_core
  grid_map_filters
  grid_map_cv
  grid_map_msgs
  rmp
  locally_reactive_controller_msgs
)

# GTSAM
# NB: The issue with including all GTSAM plugins is that it changes the names of libraries and breaks catkin packages.
# E.g. "librmp" becomes "librmpRelWithDebInfo"
set(GTSAM_BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "Enable/Disable building with all instructions supported by native architecture (binary may not be portable!)" FORCE)
find_package(GTSAM REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWitDebInfo)
endif()


###################################
## Catkin Specific Configuration ##
###################################

# Dynamic reconfigure
generate_dynamic_reconfigure_options(dynamic_reconfigure/LocallyReactiveController.cfg
                                     dynamic_reconfigure/RmpController.cfg
                                     dynamic_reconfigure/FalcoController.cfg)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES LocallyReactiveController.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs  # Or other packages containing msgs
   actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS eigen_conversions
                 roscpp
                 grid_map_core
                 grid_map_filters
                 grid_map_cv
                 grid_map_msgs
                 rmp
                 actionlib_msgs
                 locally_reactive_controller_msgs
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
)

######################################################
add_library(${PROJECT_NAME}
            src/locally_reactive_controller/manager.cpp
            src/locally_reactive_controller/visualizer.cpp
            src/locally_reactive_controller/controllers/controller_base.cpp
            src/locally_reactive_controller/controllers/trackline_controller.cpp
            src/locally_reactive_controller/controllers/falco_controller.cpp
            src/locally_reactive_controller/controllers/rmp_controller.cpp
            src/locally_reactive_controller/utils/profiler.cpp
            src/locally_reactive_controller/utils/timer.cpp
            
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS}
                                 ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      gtsam
                      yaml-cpp
                      )


add_executable(${PROJECT_NAME}_node
                 src/locally_reactive_controller/node.cpp )
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} locally_reactive_controller)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

#############
## Install ##
#############

install(TARGETS ${TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

############
## Python ##
############
catkin_install_python(PROGRAMS python/carrot_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)