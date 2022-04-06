cmake_minimum_required(VERSION 3.0.2)
project(locally_reactive_controller_msgs)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)


###################################
## Catkin Specific Configuration ##
###################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Status.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
