#pragma once
#include <field_local_planners_plugins/FalcoConfig.h>
#include <field_local_planners/local_planners/falco.hpp>
#include <field_local_planners_plugins/base_plugin.hpp>

namespace field_local_planners {

class FalcoPlugin : public BasePlugin {
 public:
  FalcoPlugin();

  void loadParameters(ros::NodeHandle& nh);
  void setupRos(ros::NodeHandle& nh);
  void dynamicReconfigureCallback(FalcoConfig& config, uint32_t level);

 private:
  // Dynamic reconfigure
  dynamic_reconfigure::Server<FalcoConfig> dynamic_reconfigure_server_;
  dynamic_reconfigure::Server<FalcoConfig>::CallbackType dynamic_reconfigure_callback_;
};

}  // namespace field_local_planners
