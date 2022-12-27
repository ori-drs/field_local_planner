cmake_minimum_required(VERSION 3.0.2)
project(field_local_planner_apf_plugin)
set (CMAKE_CXX_STANDARD 14)

set(CATKIN_PACKAGE_LIST
  dynamic_reconfigure
  field_local_planner_base_plugin
  field_local_planner_rmp
  pluginlib
  roscpp
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_LIST}
)
