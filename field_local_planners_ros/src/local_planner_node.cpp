#include <ros/console.h>
#include <ros/ros.h>
#include <field_local_planners_ros/local_planner_handler.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "field_local_planners_ros");
  ros::NodeHandle nh("~");

  field_local_planners::LocalPlannerHandler handler(nh);
  ros::spin();
  return 0;
}