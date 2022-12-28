#include <pluginlib/class_list_macros.h>
#include <field_local_planner_trackline_plugin/trackline_plugin.hpp>

PLUGINLIB_EXPORT_CLASS(field_local_planner::TracklinePlugin, field_local_planner::BasePlugin)

namespace field_local_planner {

TracklinePlugin::TracklinePlugin() : BasePlugin() {
  local_planner_ = std::make_shared<Trackline>();
}

void TracklinePlugin::loadParameters(ros::NodeHandle& nh) {
  Trackline::Parameters p;
  p.requires_sensing = utils::getParameter<bool>(nh, "requires_sensing");
  p.base_inverted = utils::getParameter<bool>(nh, "base_inverted");
  p.differential_mode = utils::getParameter<bool>(nh, "differential_mode");
  p.control_rate = utils::getParameter<double>(nh, "control_rate");
  p.robot_length = utils::getParameter<double>(nh, "robot_length");
  p.robot_width = utils::getParameter<double>(nh, "robot_width");
  p.robot_height = utils::getParameter<double>(nh, "robot_height");
  p.distance_to_goal_thr = utils::getParameter<double>(nh, "distance_to_goal_thr");
  p.orientation_to_goal_thr = utils::getParameter<double>(nh, "orientation_to_goal_thr");
  p.max_linear_velocity_x = utils::getParameter<double>(nh, "max_linear_velocity_x");
  p.max_linear_velocity_y = utils::getParameter<double>(nh, "max_linear_velocity_y");
  p.max_angular_velocity_z = utils::getParameter<double>(nh, "max_angular_velocity_z");

  p.angular_gain_p = utils::getParameterDefault(nh, "trackline/angular_gain_p", 1.0);
  p.linear_gain_p = utils::getParameterDefault(nh, "trackline/linear_gain_p", 1.0);

  std::dynamic_pointer_cast<Trackline>(local_planner_)->setParameters(p);
}

void TracklinePlugin::setupRos(ros::NodeHandle& nh) {
  dynamic_reconfigure_callback_ = boost::bind(&TracklinePlugin::dynamicReconfigureCallback, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_callback_);
}

void TracklinePlugin::dynamicReconfigureCallback(TracklineConfig& config, uint32_t level) {
  Trackline::Parameters p = std::dynamic_pointer_cast<Trackline>(local_planner_)->getParameters();

  // Trackline parameters
  UPDATE_COMMON_PARAMS(robot_length)
  UPDATE_COMMON_PARAMS(robot_width)
  UPDATE_COMMON_PARAMS(robot_height)
  UPDATE_COMMON_PARAMS(distance_to_goal_thr)
  UPDATE_COMMON_PARAMS(orientation_to_goal_thr)
  UPDATE_COMMON_PARAMS(max_linear_velocity_x)
  UPDATE_COMMON_PARAMS(max_linear_velocity_y)
  UPDATE_COMMON_PARAMS(max_angular_velocity_z)
  UPDATE_COMMON_PARAMS(angular_gain_p)
  UPDATE_COMMON_PARAMS(linear_gain_p)
}

void TracklinePlugin::publishVisualizations() {
  // TODO implement
}

}  // namespace field_local_planner
