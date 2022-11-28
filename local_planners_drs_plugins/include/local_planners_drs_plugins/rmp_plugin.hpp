#pragma once
#include <local_planners_drs_plugins/base_plugin.hpp>

namespace local_planners_drs
{

class RmpPlugin : public BasePlugin
{
public:
  RmpPlugin();
  void initialize(ros::NodeHandle& nh);

};

}