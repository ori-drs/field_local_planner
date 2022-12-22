#include <field_local_planners_plugins/trackline_plugin.hpp>

namespace field_local_planners {

TracklinePlugin::TracklinePlugin() : BasePlugin() {
  local_planner_ = std::make_shared<Trackline>();
}

void TracklinePlugin::loadParameters(ros::NodeHandle& nh) {
  Trackline::Parameters p;
  p.robot_length = utils::getParameter<double>(nh, "robot_length");
  p.robot_width = utils::getParameter<double>(nh, "robot_length");
  p.robot_height = utils::getParameter<double>(nh, "robot_length");
  p.distance_to_goal_thr = utils::getParameter<double>(nh, "distance_to_goal_thr");
  p.orientation_to_goal_thr = utils::getParameter<double>(nh, "orientation_to_goal_thr");
  p.max_linear_velocity_x = utils::getParameter<double>(nh, "max_linear_velocity_x");
  p.max_linear_velocity_y = utils::getParameter<double>(nh, "max_linear_velocity_y");
  p.max_angular_velocity_z = utils::getParameter<double>(nh, "max_angular_velocity_z");

  p.angular_gain_p = utils::getParameterDefault(nh, "angular_gain_p", 1.0);
  p.linear_gain_p = utils::getParameterDefault(nh, "linear_gain_p", 1.0);
}

void TracklinePlugin::setupRos(ros::NodeHandle& nh) {
  dynamic_reconfigure_callback_ = boost::bind(&TracklinePlugin::dynamicReconfigureCallback, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_callback_);
}

void TracklinePlugin::dynamicReconfigureCallback(TracklineConfig& config, uint32_t level) {
  //
}

}  // namespace field_local_planners
