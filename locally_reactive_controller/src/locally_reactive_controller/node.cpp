#include <ros/ros.h>
#include <ros/console.h>

#include <locally_reactive_controller/manager.hpp>


int main( int argc, char** argv ){
  ros::init(argc, argv, "locally_reactive_controller");
  ros::NodeHandle nh("~");

  Manager manager(nh, ros::this_node::getName());

  ROS_INFO_STREAM("Locally Reactive Controller ros ready");
  ros::spin();
  return 0;
}
