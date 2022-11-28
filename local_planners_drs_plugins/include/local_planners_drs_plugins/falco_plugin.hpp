#pragma once
#include <local_planners_drs/local_planners/falco.hpp>
#include <local_planners_drs_plugins/base_plugin.hpp>

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

