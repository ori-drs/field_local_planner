#pragma once
#include <field_local_planner_trackline_plugin/TracklineConfig.h>
#include <field_local_planner_base_plugin/base_plugin.hpp>
#include <field_local_planner_trackline/trackline.hpp>

namespace field_local_planner {

class TracklinePlugin : public BasePlugin {
 public:
  TracklinePlugin();

  std::string getName() { return "trackline"; };
  void loadParameters(ros::NodeHandle& nh);
  void setupRos(ros::NodeHandle& nh);
  void publishVisualizations();

  void dynamicReconfigureCallback(TracklineConfig& config, uint32_t level);

 private:
  // Dynamic reconfigure
  using DynParamServer = dynamic_reconfigure::Server<TracklineConfig>;
  using DynParamCallback = dynamic_reconfigure::Server<TracklineConfig>::CallbackType;
  std::shared_ptr<DynParamServer> dynamic_reconfigure_server_;
  DynParamCallback dynamic_reconfigure_callback_;
};

}  // namespace field_local_planner