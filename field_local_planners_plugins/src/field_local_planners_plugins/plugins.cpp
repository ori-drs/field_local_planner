#include <pluginlib/class_list_macros.h>
#include <field_local_planners_plugins/base_plugin.hpp>
#include <field_local_planners_plugins/falco_plugin.hpp>
#include <field_local_planners_plugins/rmp_plugin.hpp>
#include <field_local_planners_plugins/trackline_plugin.hpp>

using namespace field_local_planners;

PLUGINLIB_EXPORT_CLASS(FalcoPlugin, BasePlugin)
PLUGINLIB_EXPORT_CLASS(RmpPlugin, BasePlugin)
PLUGINLIB_EXPORT_CLASS(TracklinePlugin, BasePlugin)