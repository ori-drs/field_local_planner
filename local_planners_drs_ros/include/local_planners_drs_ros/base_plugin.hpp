#pragma once
#include <ros/ros.h>

namespace local_planners_drs
{

class BasePlugin
{
public:
  void initialize(ros::NodeHandle& nh);
  
protected:
  BasePlugin();
};

}