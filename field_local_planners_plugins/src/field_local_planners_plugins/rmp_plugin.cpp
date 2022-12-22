#include <field_local_planners_plugins/rmp_plugin.hpp>

namespace field_local_planners {

RmpPlugin::RmpPlugin() : BasePlugin() {
  local_planner_ = std::make_shared<Rmp>();
}

void RmpPlugin::loadParameters(ros::NodeHandle& nh) {
  Rmp::Parameters p;
  p.robot_length = utils::getParameter<double>(nh, "robot_length");
  p.robot_width = utils::getParameter<double>(nh, "robot_length");
  p.robot_height = utils::getParameter<double>(nh, "robot_length");
  p.distance_to_goal_thr = utils::getParameter<double>(nh, "distance_to_goal_thr");
  p.orientation_to_goal_thr = utils::getParameter<double>(nh, "orientation_to_goal_thr");
  p.max_linear_velocity_x = utils::getParameter<double>(nh, "max_linear_velocity_x");
  p.max_linear_velocity_y = utils::getParameter<double>(nh, "max_linear_velocity_y");
  p.max_angular_velocity_z = utils::getParameter<double>(nh, "max_angular_velocity_z");

  p.config_folder = utils::getParameterDefault(nh, "rmp/config_folder", std::string("/local_planners/rmp/"));
  p.sphere_radius_factor = utils::getParameterDefault(nh, "rmp/sphere_radius_factor", 1.0);
  p.geodesic_goal_gain = utils::getParameterDefault(nh, "rmp/geodesic_goal_gain", 7.0);
  p.geodesic_goal_offset = utils::getParameterDefault(nh, "rmp/geodesic_goal_offset", 0.1);
  p.geodesic_goal_steepness = utils::getParameterDefault(nh, "rmp/geodesic_goal_steepness", 7.0);
  p.geodesic_heading_gain = utils::getParameterDefault(nh, "rmp/geodesic_heading_gain", 7.0);
  p.geodesic_heading_offset = utils::getParameterDefault(nh, "rmp/geodesic_heading_offset", 0.1);
  p.geodesic_heading_steepness = utils::getParameterDefault(nh, "rmp/geodesic_heading_steepness", 1.0);
  p.goal_position_gain = utils::getParameterDefault(nh, "rmp/goal_position_gain", 2.0);
  p.goal_position_offset = utils::getParameterDefault(nh, "rmp/goal_position_offset", 0.1);
  p.goal_position_steepness = utils::getParameterDefault(nh, "rmp/goal_position_steepness", 1.0);
  p.goal_heading_gain = utils::getParameterDefault(nh, "rmp/goal_heading_gain", 2.0);
  p.goal_heading_offset = utils::getParameterDefault(nh, "rmp/goal_heading_offset", 0.1);
  p.goal_heading_steepness = utils::getParameterDefault(nh, "rmp/goal_heading_steepness", 1.0);
  p.damping = utils::getParameterDefault(nh, "rmp/damping", 1.0);
  p.obstacle_gain = utils::getParameterDefault(nh, "rmp/obstacle_gain", 1.0);
  p.obstacle_offset = utils::getParameterDefault(nh, "rmp/obstacle_offset", 0.1);
  p.obstacle_steepness = utils::getParameterDefault(nh, "rmp/obstacle_steepness", 1.0);
  p.heading_gain = utils::getParameterDefault(nh, "rmp/heading_gain", 1.0);
  p.heading_offset = utils::getParameterDefault(nh, "rmp/heading_offset", 1.0);
  p.heading_steepness = utils::getParameterDefault(nh, "rmp/heading_steepness", 1.0);
  p.regularization = utils::getParameterDefault(nh, "rmp/regularization", 0.1);
  p.integration_time = utils::getParameterDefault(nh, "rmp/integration_time", 1.0);

  p.geodesic_goal_weight = utils::getParameterDefault(nh, "rmp/geodesic_goal_weight", 1.0);
  p.geodesic_heading_weight = utils::getParameterDefault(nh, "rmp/geodesic_heading_weight", 1.0);
  p.goal_position_weight = utils::getParameterDefault(nh, "rmp/goal_position_weight", 1.0);
  p.goal_heading_weight = utils::getParameterDefault(nh, "rmp/goal_heading_weight", 1.0);
  p.obstacle_weight = utils::getParameterDefault(nh, "rmp/obstacle_weight", 1.0);
  p.damping_weight = utils::getParameterDefault(nh, "rmp/damping_weight", 1.0);
  p.heading_weight = utils::getParameterDefault(nh, "rmp/heading_weight", 1.0);

  local_planner_->initialize(p);
}

void RmpPlugin::setupRos(ros::NodeHandle& nh) {
  dynamic_reconfigure_callback_ = boost::bind(&RmpPlugin::dynamicReconfigureCallback, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_callback_);
}

void RmpPlugin::dynamicReconfigureCallback(RmpConfig& config, uint32_t level) {
  //
}

}  // namespace field_local_planners
