cmake_minimum_required(VERSION 3.0.2)
project(field_local_planner_msgs)
set(CMAKE_CXX_STANDARD 14)

set(MSGS_DEPENDENCIES_LIST
  actionlib_msgs
  geometry_msgs
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${MSGS_DEPENDENCIES_LIST}
  message_generation
)


###################################
## Catkin Specific Configuration ##
###################################

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES NewGoal.action
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Status.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
     actionlib_msgs
     geometry_msgs
     std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS
    ${MSGS_DEPENDENCIES_LIST}
    message_runtime
)

#############
## Install ##
#############

install(TARGETS ${TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)