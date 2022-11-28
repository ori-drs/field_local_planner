#include <pluginlib/class_list_macros.h>
#include <local_planners_drs_plugins/base_plugin.hpp>
#include <local_planners_drs_plugins/falco_plugin.hpp>
#include <local_planners_drs_plugins/rmp_plugin.hpp>
#include <local_planners_drs_plugins/trackline_plugin.hpp>

PLUGINLIB_EXPORT_CLASS(local_planners_drs::FalcoPlugin, local_planners_drs::BasePlugin)
PLUGINLIB_EXPORT_CLASS(local_planners_drs::RmpPlugin, local_planners_drs::BasePlugin)
PLUGINLIB_EXPORT_CLASS(local_planners_drs::TracklinePlugin, local_planners_drs::BasePlugin)