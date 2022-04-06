cmake_minimum_required(VERSION 3.0.2)
project(locally_reactive_controller_evaluation)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  rospy
  dynamic_reconfigure
  actionlib
  actionlib_msgs
  message_generation
  geometry_msgs
  std_msgs
  nav_msgs
  locally_reactive_controller_msgs
  # packages below are required for simulation experiments only
  gazebo_msgs
  any_gazebo_msgs
  std_srvs
  locally_reactive_controller_anymal
)

###################################
## Catkin Specific Configuration ##
###################################
# Python
catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS locally_reactive_controller_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  gazebo_msgs
  any_gazebo_msgs
  std_srvs
  locally_reactive_controller_anymal
)

############
## Python ##
############
catkin_install_python(PROGRAMS src/anymal_simulation_experiment.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)