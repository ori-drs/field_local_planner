#include <pluginlib/class_list_macros.h>
#include <field_local_planner_rmp_plugin/rmp_plugin.hpp>

PLUGINLIB_EXPORT_CLASS(field_local_planner::RmpPlugin, field_local_planner::BasePlugin)

namespace field_local_planner {

RmpPlugin::RmpPlugin() : BasePlugin() {
  local_planner_ = std::make_shared<Rmp>();
}

void RmpPlugin::loadParameters(ros::NodeHandle& nh) {
  Rmp::Parameters p;
  p.requires_sensing = utils::getParameter<bool>(nh, "requires_sensing");
  p.robot_length = utils::getParameter<double>(nh, "robot_length");
  p.robot_width = utils::getParameter<double>(nh, "robot_width");
  p.robot_height = utils::getParameter<double>(nh, "robot_height");
  p.distance_to_goal_thr = utils::getParameter<double>(nh, "distance_to_goal_thr");
  p.orientation_to_goal_thr = utils::getParameter<double>(nh, "orientation_to_goal_thr");
  p.max_linear_velocity_x = utils::getParameter<double>(nh, "max_linear_velocity_x");
  p.max_linear_velocity_y = utils::getParameter<double>(nh, "max_linear_velocity_y");
  p.max_angular_velocity_z = utils::getParameter<double>(nh, "max_angular_velocity_z");

  p.sphere_radius_factor = utils::getParameterDefault(nh, "rmp/sphere_radius_factor", 1.0);
  p.integration_time = utils::getParameterDefault(nh, "rmp/integration_time", 1.0);

  // Read RMP parameters
  for (auto rmp_name : std::dynamic_pointer_cast<Rmp>(local_planner_)->getAvailableRmps()) {
    Rmp::RmpParameters rmp_p;
    rmp_p.name = rmp_name;
    rmp_p.weight = utils::getParameterDefault(nh, "rmp/" + rmp_p.name + "/weight", 1.0);
    rmp_p.gain = utils::getParameterDefault(nh, "rmp/" + rmp_p.name + "/gain", 1.0);
    rmp_p.metric_type = utils::getParameterDefault<std::string>(nh, "rmp/" + rmp_p.name + "/metric/type", "sigmoid");
    rmp_p.metric_offset = utils::getParameterDefault(nh, "rmp/" + rmp_p.name + "/metric/offset", 1.0);
    rmp_p.metric_steepness = utils::getParameterDefault(nh, "rmp/" + rmp_p.name + "/metric/steepness", 1.0);

    std::vector<double> color = utils::getParameterVector<double>(nh, "rmp/" + rmp_p.name + "/color");
    rmp_p.color(0) = color[0];
    rmp_p.color(1) = color[1];
    rmp_p.color(2) = color[2];

    p.rmp_parameters[rmp_name] = rmp_p;
  }
  std::dynamic_pointer_cast<Rmp>(local_planner_)->setParameters(p);

  // Prepare control points
  XmlRpc::XmlRpcValue control_points_list;
  nh.getParam("rmp/control_points", control_points_list);

  Rmp::ControlPoints control_points;

  for (size_t i = 0; i < control_points_list.size(); ++i) {
    XmlRpc::XmlRpcValue node = control_points_list[i];

    Rmp::ControlPoint cp;
    cp.id = static_cast<std::string>(node["id"]);
    cp.radius = static_cast<double>(node["radius"]);
    cp.inflated_radius = cp.radius * p.sphere_radius_factor;
    cp.point_factor(0) = static_cast<double>(node["point_factor"][0]);
    cp.point_factor(1) = static_cast<double>(node["point_factor"][1]);
    cp.position(0) = cp.point_factor(0) * p.robot_length * 0.5;
    cp.position(1) = cp.point_factor(1) * p.robot_width * 0.5;
    cp.color(0) = static_cast<double>(node["color"][0]);
    cp.color(1) = static_cast<double>(node["color"][1]);
    cp.color(2) = static_cast<double>(node["color"][2]);

    XmlRpc::XmlRpcValue affected_by_list = node["affected_by"];
    for (size_t j = 0; j < affected_by_list.size(); ++j) {
      cp.affected_by.push_back(static_cast<std::string>(affected_by_list[j]));
    }
    control_points.push_back(cp);
  }
  std::dynamic_pointer_cast<Rmp>(local_planner_)->setControlPoints(control_points);
}

void RmpPlugin::setupRos(ros::NodeHandle& nh) {
  dynamic_reconfigure_callback_ = boost::bind(&RmpPlugin::dynamicReconfigureCallback, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_callback_);
}

void RmpPlugin::dynamicReconfigureCallback(RmpConfig& config, uint32_t level) {
  Rmp::Parameters p = std::dynamic_pointer_cast<Rmp>(local_planner_)->getParameters();
  Rmp::ControlPoints cps = std::dynamic_pointer_cast<Rmp>(local_planner_)->getControlPoints();

#define UPDATE_COMMON_PARAMS(VAR) utils::assignAndPrintDiff(#VAR, p.VAR, config.VAR);
#define UPDATE_RMP_PARAMS(RMP, VAR) utils::assignAndPrintDiff(#RMP "_" #VAR, p.rmp_parameters[#RMP].VAR, config.RMP##_##VAR);

#define UPDATE_RMP(RMP)                 \
  UPDATE_RMP_PARAMS(RMP, weight)        \
  UPDATE_RMP_PARAMS(RMP, gain)          \
  UPDATE_RMP_PARAMS(RMP, metric_type)   \
  UPDATE_RMP_PARAMS(RMP, metric_offset) \
  UPDATE_RMP_PARAMS(RMP, metric_steepness)

  // RMP parameters
  UPDATE_COMMON_PARAMS(robot_length)
  UPDATE_COMMON_PARAMS(robot_width)
  UPDATE_COMMON_PARAMS(robot_height)
  UPDATE_COMMON_PARAMS(distance_to_goal_thr)
  UPDATE_COMMON_PARAMS(orientation_to_goal_thr)
  UPDATE_COMMON_PARAMS(max_linear_velocity_x)
  UPDATE_COMMON_PARAMS(max_linear_velocity_y)
  UPDATE_COMMON_PARAMS(max_angular_velocity_z)
  UPDATE_COMMON_PARAMS(sphere_radius_factor)
  UPDATE_COMMON_PARAMS(integration_time)

  UPDATE_RMP(geodesic_goal)
  UPDATE_RMP(geodesic_heading)
  UPDATE_RMP(goal_position)
  UPDATE_RMP(goal_orientation)
  UPDATE_RMP(velocity_heading)
  UPDATE_RMP(damping)
  UPDATE_RMP(sdf_obstacle)
  UPDATE_RMP(regularization)

  // Update control points
  for (auto& cp : cps) {
    cp.inflated_radius = cp.radius * p.sphere_radius_factor;
    cp.position(0) = cp.point_factor(0) * p.robot_length * 0.5;
    cp.position(1) = cp.point_factor(1) * p.robot_width * 0.5;
  }

  std::dynamic_pointer_cast<Rmp>(local_planner_)->setParameters(p);
  std::dynamic_pointer_cast<Rmp>(local_planner_)->setControlPoints(cps);
}

}  // namespace field_local_planner
