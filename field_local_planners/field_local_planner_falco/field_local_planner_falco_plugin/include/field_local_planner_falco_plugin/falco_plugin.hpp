#pragma once
#include <field_local_planner_falco_plugin/FalcoConfig.h>
#include <field_local_planner_base_plugin/base_plugin.hpp>
#include <field_local_planner_falco/falco_local_planner.hpp>

namespace field_local_planner {

class FalcoPlugin : public BasePlugin {
 public:
  FalcoPlugin();

  std::string getName() { return "falco"; };
  void loadParameters(ros::NodeHandle& nh);
  void setupRos(ros::NodeHandle& nh);
  void publishVisualizations();

  void dynamicReconfigureCallback(FalcoConfig& config, uint32_t level);

 private:
  // Dynamic reconfigure
  using DynParamServer = dynamic_reconfigure::Server<FalcoConfig>;
  using DynParamCallback = dynamic_reconfigure::Server<FalcoConfig>::CallbackType;
  std::shared_ptr<DynParamServer> dynamic_reconfigure_server_;
  DynParamCallback dynamic_reconfigure_callback_;

  // Visualization publisher
  ros::Publisher free_paths_pub_;
  ros::Publisher collision_map_pub_;
};

}  // namespace field_local_planner
