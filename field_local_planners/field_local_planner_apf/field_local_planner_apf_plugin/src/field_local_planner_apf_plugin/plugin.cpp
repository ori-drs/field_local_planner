#include <field_local_planner_apf_plugin/plugin.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(field_local_planner::ApfPlugin, field_local_planner::BasePlugin)

namespace field_local_planner {

ApfPlugin::ApfPlugin() : BasePlugin() {
  // We make an instance of the Artificial Potential Field local planner
  // as a RMP one but with fixed metrics
  local_planner_ = std::make_shared<Rmp>();
}

void ApfPlugin::loadParameters(ros::NodeHandle& nh) {
  Rmp::Parameters p;
  p.robot_length = utils::getParameter<double>(nh, "robot_length");
  p.robot_width = utils::getParameter<double>(nh, "robot_length");
  p.robot_height = utils::getParameter<double>(nh, "robot_length");
  p.distance_to_goal_thr = utils::getParameter<double>(nh, "distance_to_goal_thr");
  p.orientation_to_goal_thr = utils::getParameter<double>(nh, "orientation_to_goal_thr");
  p.max_linear_velocity_x = utils::getParameter<double>(nh, "max_linear_velocity_x");
  p.max_linear_velocity_y = utils::getParameter<double>(nh, "max_linear_velocity_y");
  p.max_angular_velocity_z = utils::getParameter<double>(nh, "max_angular_velocity_z");

  p.config_folder = utils::getParameterDefault(nh, "apf/config_folder", std::string("/local_planners/rmp/"));
  p.sphere_radius_factor = utils::getParameterDefault(nh, "apf/sphere_radius_factor", 1.0);
  p.geodesic_goal_gain = utils::getParameterDefault(nh, "apf/geodesic_goal_gain", 7.0);
  p.geodesic_goal_offset = utils::getParameterDefault(nh, "apf/geodesic_goal_offset", 0.1);
  p.geodesic_goal_steepness = utils::getParameterDefault(nh, "apf/geodesic_goal_steepness", 7.0);
  p.geodesic_heading_gain = utils::getParameterDefault(nh, "apf/geodesic_heading_gain", 7.0);
  p.geodesic_heading_offset = utils::getParameterDefault(nh, "apf/geodesic_heading_offset", 0.1);
  p.geodesic_heading_steepness = utils::getParameterDefault(nh, "apf/geodesic_heading_steepness", 1.0);
  p.goal_position_gain = utils::getParameterDefault(nh, "apf/goal_position_gain", 2.0);
  p.goal_position_offset = utils::getParameterDefault(nh, "apf/goal_position_offset", 0.1);
  p.goal_position_steepness = utils::getParameterDefault(nh, "apf/goal_position_steepness", 1.0);
  p.goal_heading_gain = utils::getParameterDefault(nh, "apf/goal_heading_gain", 2.0);
  p.goal_heading_offset = utils::getParameterDefault(nh, "apf/goal_heading_offset", 0.1);
  p.goal_heading_steepness = utils::getParameterDefault(nh, "apf/goal_heading_steepness", 1.0);
  p.damping = utils::getParameterDefault(nh, "apf/damping", 1.0);
  p.obstacle_gain = utils::getParameterDefault(nh, "apf/obstacle_gain", 1.0);
  p.obstacle_offset = utils::getParameterDefault(nh, "apf/obstacle_offset", 0.1);
  p.obstacle_steepness = utils::getParameterDefault(nh, "apf/obstacle_steepness", 1.0);
  p.heading_gain = utils::getParameterDefault(nh, "apf/heading_gain", 1.0);
  p.heading_offset = utils::getParameterDefault(nh, "apf/heading_offset", 1.0);
  p.heading_steepness = utils::getParameterDefault(nh, "apf/heading_steepness", 1.0);
  p.regularization = utils::getParameterDefault(nh, "apf/regularization", 0.1);
  p.integration_time = utils::getParameterDefault(nh, "apf/integration_time", 1.0);

  p.geodesic_goal_weight = utils::getParameterDefault(nh, "apf/geodesic_goal_weight", 1.0);
  p.geodesic_heading_weight = utils::getParameterDefault(nh, "apf/geodesic_heading_weight", 1.0);
  p.goal_position_weight = utils::getParameterDefault(nh, "apf/goal_position_weight", 1.0);
  p.goal_heading_weight = utils::getParameterDefault(nh, "apf/goal_heading_weight", 1.0);
  p.obstacle_weight = utils::getParameterDefault(nh, "apf/obstacle_weight", 1.0);
  p.damping_weight = utils::getParameterDefault(nh, "apf/damping_weight", 1.0);
  p.heading_weight = utils::getParameterDefault(nh, "apf/heading_weight", 1.0);

  local_planner_->initialize(p);
}

void ApfPlugin::setupRos(ros::NodeHandle& nh) {
  dynamic_reconfigure_callback_ = boost::bind(&ApfPlugin::dynamicReconfigureCallback, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_callback_);
}

void ApfPlugin::dynamicReconfigureCallback(ApfConfig& config, uint32_t level) {
  //
}

}  // namespace field_local_planner
