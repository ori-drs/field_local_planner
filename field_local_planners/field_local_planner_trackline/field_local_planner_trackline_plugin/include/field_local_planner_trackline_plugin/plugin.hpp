#pragma once
#include <field_local_planner_trackline_plugin/TracklineConfig.h>
#include <field_local_planner_base_plugin/plugin.hpp>
#include <field_local_planner_trackline/trackline.hpp>

namespace field_local_planner {

class TracklinePlugin : public BasePlugin {
 public:
  TracklinePlugin();

  void loadParameters(ros::NodeHandle& nh);
  void setupRos(ros::NodeHandle& nh);
  void dynamicReconfigureCallback(TracklineConfig& config, uint32_t level);

 private:
  // Dynamic reconfigure
  dynamic_reconfigure::Server<TracklineConfig> dynamic_reconfigure_server_;
  dynamic_reconfigure::Server<TracklineConfig>::CallbackType dynamic_reconfigure_callback_;
};

}  // namespace field_local_planner