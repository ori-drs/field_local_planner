#include <ros/console.h>
#include <ros/ros.h>
#include <field_local_planner_ros/local_planner_ros.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "field_local_planner_ros");
  ros::NodeHandle nh("~");

  field_local_planner::LocalPlannerRos handler(nh);
  return 0;
}