#pragma once
#include <local_planners_drs_plugins/base_plugin.hpp>

namespace local_planners_drs
{

class TracklinePlugin : public BasePlugin
{
public:
  TracklinePlugin();
  void initialize(ros::NodeHandle& nh);

};

}