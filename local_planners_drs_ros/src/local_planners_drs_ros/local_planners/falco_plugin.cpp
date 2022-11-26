#include <local_planners_drs_ros/local_planners/falco_plugin.hpp>

namespace local_planners_drs
{

FalcoPlugin::FalcoPlugin()
: BasePlugin()
{}

  
} // namespace local_planners_drs

PLUGINLIB_EXPORT_CLASS(local_planners_drs::FalcoPlugin, local_planners_drs::BasePlugin)