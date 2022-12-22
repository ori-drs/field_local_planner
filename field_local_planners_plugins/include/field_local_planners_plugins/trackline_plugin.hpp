#pragma once
#include <field_local_planners_plugins/TracklineConfig.h>
#include <field_local_planners/local_planners/trackline.hpp>
#include <field_local_planners_plugins/base_plugin.hpp>

namespace field_local_planners {

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

}  // namespace field_local_planners