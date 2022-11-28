#pragma once
#include <ros/ros.h>

namespace local_planners_drs
{

class BasePlugin
{
protected:
  BasePlugin()
  {
    
  }

public:
  void initialize(ros::NodeHandle& nh)
  {
    ROS_INFO("Initialize");
  }
  

};

}
