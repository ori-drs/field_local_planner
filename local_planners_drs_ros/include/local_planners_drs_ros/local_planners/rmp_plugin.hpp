#pragma once
#include <local_planners_drs_ros/base_plugin.hpp>
#include <pluginlib/class_list_macros.h>

namespace local_planners_drs
{

class RmpPlugin : public BasePlugin
{
public:
  RmpPlugin();
  void initialize(ros::NodeHandle& nh);

};

}