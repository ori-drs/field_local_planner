#pragma once
#include <field_local_planner_apf_plugin/ApfConfig.h>
#include <field_local_planner_base_plugin/plugin.hpp>
#include <field_local_planner_rmp/rmp.hpp>

namespace field_local_planner {

class ApfPlugin : public BasePlugin {
 public:
  ApfPlugin();

  void loadParameters(ros::NodeHandle& nh);
  void setupRos(ros::NodeHandle& nh);
  void dynamicReconfigureCallback(ApfConfig& config, uint32_t level);

 private:
  // Dynamic reconfigure
  dynamic_reconfigure::Server<ApfConfig> dynamic_reconfigure_server_;
  dynamic_reconfigure::Server<ApfConfig>::CallbackType dynamic_reconfigure_callback_;
};

}  // namespace field_local_planner