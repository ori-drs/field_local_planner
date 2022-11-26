#pragma once
#include <local_planners_drs_ros/base_plugin.hpp>
#include <pluginlib/class_list_macros.h>
#include <local_planners_drs/local_planners/falco.hpp>

namespace local_planners_drs
{

class FalcoPlugin : public BasePlugin
{
public:
  FalcoPlugin();
  // void initialize(ros::NodeHandle& nh);

private:
  Falco planner_;

};

}