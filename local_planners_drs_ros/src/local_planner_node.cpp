#include <pluginlib/class_loader.h>
#include <local_planners_drs_plugins/base_plugin.hpp>
#include <ros/ros.h>
#include <ros/console.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_planners_drs_ros");
  ros::NodeHandle nh("~");

  // Prepare loader
  pluginlib::ClassLoader<local_planners_drs::BasePlugin> plugin_loader("local_planners_drs_plugins", "local_planners_drs::BasePlugin");

  try
  {
    boost::shared_ptr<local_planners_drs::BasePlugin> local_planner = plugin_loader.createInstance("local_planners_drs::FalcoPlugin");
    local_planner->initialize(nh);
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    return -1;
  }


  ros::spin();
  return 0;
}