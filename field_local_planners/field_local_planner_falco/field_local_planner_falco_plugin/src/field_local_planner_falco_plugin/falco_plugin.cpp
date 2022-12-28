#include <pluginlib/class_list_macros.h>
#include <field_local_planner_falco_plugin/falco_plugin.hpp>

PLUGINLIB_EXPORT_CLASS(field_local_planner::FalcoPlugin, field_local_planner::BasePlugin)

namespace field_local_planner {

FalcoPlugin::FalcoPlugin() : BasePlugin() {
  local_planner_ = std::make_shared<Falco>();
}

void FalcoPlugin::loadParameters(ros::NodeHandle& nh) {
  // Load parameters from parameter server
  Falco::Parameters p;
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

  p.config_folder = utils::getParameterDefault(nh, "falco/config_folder", std::string("config/"));
  p.goal_clearance = utils::getParameterDefault(nh, "falco/goal_clearance", 0.1);
  p.robot_clearance = utils::getParameterDefault(nh, "falco/robot_clearance", 0.1);
  p.sensor_range = utils::getParameterDefault(nh, "falco/sensor_range", 2.0);
  p.linear_gain_p = utils::getParameterDefault(nh, "falco/linear_gain_p", 1.0);
  p.angular_gain_p = utils::getParameterDefault(nh, "falco/angular_gain_p", 1.0);
  p.linear_acceleration = utils::getParameterDefault(nh, "falco/linear_acceleration", 0.0001);
  p.path_scale = utils::getParameterDefault(nh, "falco/path_scale", 1.25);
  p.path_scale_step = utils::getParameterDefault(nh, "falco/path_scale_step", 0.25);
  p.min_path_scale = utils::getParameterDefault(nh, "falco/min_path_scale", 0.75);
  p.path_range_step = utils::getParameterDefault(nh, "falco/path_range_step", 0.5);
  p.min_path_range = utils::getParameterDefault(nh, "falco/min_path_range", 1.0);
  p.look_ahead_distance = utils::getParameterDefault(nh, "falco/look_ahead_distance", 0.5);
  p.differential_mode_cost = utils::getParameterDefault(nh, "falco/differential_mode_cost", 1.0);
  p.goal_cost = utils::getParameterDefault(nh, "falco/goal_cost", 1.0);
  p.traversability_cost = utils::getParameterDefault(nh, "falco/traversability_cost", 1.0);
  p.traversable_thr = utils::getParameterDefault(nh, "falco/traversable_thr", 0.5);
  p.point_per_path_thr = utils::getParameterDefault(nh, "falco/point_per_path_thr", 2);
  p.check_rotational_collisions = utils::getParameterDefault(nh, "falco/check_rotational_collisions", true);
  p.use_path_crop_by_goal = utils::getParameterDefault(nh, "falco/use_path_crop_by_goal", false);
  

  // Initialize local planner
  std::dynamic_pointer_cast<Falco>(local_planner_)->setParameters(p);
}

void FalcoPlugin::setupRos(ros::NodeHandle& nh) {
  dynamic_reconfigure_callback_ = boost::bind(&FalcoPlugin::dynamicReconfigureCallback, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_callback_);
}

void FalcoPlugin::dynamicReconfigureCallback(FalcoConfig& config, uint32_t level) {
  Falco::Parameters p = std::dynamic_pointer_cast<Falco>(local_planner_)->getParameters();

  // Falco parameters
  UPDATE_COMMON_PARAMS(robot_length)
  UPDATE_COMMON_PARAMS(robot_width)
  UPDATE_COMMON_PARAMS(robot_height)
  UPDATE_COMMON_PARAMS(distance_to_goal_thr)
  UPDATE_COMMON_PARAMS(orientation_to_goal_thr)
  UPDATE_COMMON_PARAMS(max_linear_velocity_x)
  UPDATE_COMMON_PARAMS(max_linear_velocity_y)
  UPDATE_COMMON_PARAMS(max_angular_velocity_z)
}

void FalcoPlugin::publishVisualizations() {
  // TODO implement
}

}  // namespace field_local_planner
