#include <pluginlib/class_loader.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <field_local_planner_base_plugin/base_plugin.hpp>
#include <field_local_planner_base_plugin/utils.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "field_local_planner");
  ros::NodeHandle nh("~");

  // Get name of plugin from parameter server
  std::string plugin_name = field_local_planner::utils::getParameter<std::string>(nh, "local_planner");

  // Prepare loader
  pluginlib::ClassLoader<field_local_planner::BasePlugin> plugin_loader_("field_local_planner_base_plugin",
                                                                         "field_local_planner::BasePlugin");

  boost::shared_ptr<field_local_planner::BasePlugin> local_planner_plugin_;  // The local planner plugin, needs to be boost:shared_ptr
  try {
    // Create instance
    local_planner_plugin_ = plugin_loader_.createInstance(plugin_name);

    // Load parameters
    local_planner_plugin_->initialize(nh);

  } catch (pluginlib::PluginlibException& ex) {
    ROS_FATAL_STREAM("The plugin [" << plugin_name << "] failed to load for some reason. Error: " << ex.what());
    exit(-1);
  }

  return 0;
}