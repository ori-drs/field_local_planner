#pragma once
#include <local_planners_drs_ros/base_plugin.hpp>
#include <pluginlib/class_list_macros.h>

namespace local_planners_drs
{

class TracklinePlugin : public BasePlugin
{
public:
  TracklinePlugin();
  void initialize(ros::NodeHandle& nh);

};

}