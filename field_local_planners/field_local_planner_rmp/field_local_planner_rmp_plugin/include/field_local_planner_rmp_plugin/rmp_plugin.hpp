#pragma once
#include <field_local_planner_rmp_plugin/RmpConfig.h>
#include <field_local_planner_base_plugin/base_plugin.hpp>
#include <field_local_planner_rmp/rmp_local_planner.hpp>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace field_local_planner {

class RmpPlugin : public BasePlugin {
 public:
  RmpPlugin();

  void loadParameters(ros::NodeHandle& nh);
  void setupRos(ros::NodeHandle& nh);
  void publishVisualizations();

  void dynamicReconfigureCallback(RmpConfig& config, uint32_t level);

 private:
  // Dynamic reconfigure
  dynamic_reconfigure::Server<RmpConfig> dynamic_reconfigure_server_;
  dynamic_reconfigure::Server<RmpConfig>::CallbackType dynamic_reconfigure_callback_;

  // Visualization publisher
  ros::Publisher control_points_pub_;
};

}  // namespace field_local_planner