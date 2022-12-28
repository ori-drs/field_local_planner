#pragma once
#include <field_local_planner_falco_plugin/FalcoConfig.h>
#include <field_local_planner_base_plugin/base_plugin.hpp>
#include <field_local_planner_falco/falco.hpp>

namespace field_local_planner {

class FalcoPlugin : public BasePlugin {
 public:
  FalcoPlugin();

  void loadParameters(ros::NodeHandle& nh);
  void setupRos(ros::NodeHandle& nh);
  void publishVisualizations();

  void dynamicReconfigureCallback(FalcoConfig& config, uint32_t level);

 private:
  // Dynamic reconfigure
  dynamic_reconfigure::Server<FalcoConfig> dynamic_reconfigure_server_;
  dynamic_reconfigure::Server<FalcoConfig>::CallbackType dynamic_reconfigure_callback_;
};

}  // namespace field_local_planner
