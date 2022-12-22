#pragma once
#include <field_local_planners_plugins/RmpConfig.h>
#include <field_local_planners/local_planners/rmp.hpp>
#include <field_local_planners_plugins/base_plugin.hpp>

namespace field_local_planners {

class RmpPlugin : public BasePlugin {
 public:
  RmpPlugin();

  void loadParameters(ros::NodeHandle& nh);
  void setupRos(ros::NodeHandle& nh);
  void dynamicReconfigureCallback(RmpConfig& config, uint32_t level);

 private:
  // Dynamic reconfigure
  dynamic_reconfigure::Server<RmpConfig> dynamic_reconfigure_server_;
  dynamic_reconfigure::Server<RmpConfig>::CallbackType dynamic_reconfigure_callback_;
};

}  // namespace field_local_planners