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

  p.sphere_radius_factor = utils::getParameterDefault(nh, "rmp/sphere_radius_factor", 1.0);
  p.integration_time = utils::getParameterDefault(nh, "rmp/integration_time", 1.0);

  // Read RMP parameters
  for (auto rmp_name : std::dynamic_pointer_cast<Rmp>(local_planner_)->getAvailableRmps()) {
    Rmp::RmpParameters rmp_p;
    rmp_p.name = rmp_name;
    rmp_p.weight = utils::getParameterDefault(nh, "rmp/policies/" + rmp_p.name + "/weight", 1.0);
    rmp_p.gain = utils::getParameterDefault(nh, "rmp/policies/" + rmp_p.name + "/gain", 1.0);
    rmp_p.metric_type = utils::getParameterDefault<std::string>(nh, "rmp/policies/" + rmp_p.name + "/metric/type", "sigmoid");
    rmp_p.metric_offset = utils::getParameterDefault(nh, "rmp/policies/" + rmp_p.name + "/metric/offset", 1.0);
    rmp_p.metric_steepness = utils::getParameterDefault(nh, "rmp/policies/" + rmp_p.name + "/metric/steepness", 1.0);

    std::vector<double> color = utils::getParameterVector<double>(nh, "rmp/policies/" + rmp_p.name + "/color");
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

  control_points_pub_ = nh.advertise<visualization_msgs::MarkerArray>(ros::this_node::getName() + "/rmp/control_points", 10);
}

void RmpPlugin::dynamicReconfigureCallback(RmpConfig& config, uint32_t level) {
#define UPDATE_RMP_PARAMS(RMP, VAR) utils::assignAndPrintDiff(#RMP "_" #VAR, p.rmp_parameters[#RMP].VAR, config.RMP##_##VAR);

#define UPDATE_RMP(RMP)                 \
  UPDATE_RMP_PARAMS(RMP, weight)        \
  UPDATE_RMP_PARAMS(RMP, gain)          \
  // UPDATE_RMP_PARAMS(RMP, metric_type)   \
  // UPDATE_RMP_PARAMS(RMP, metric_offset) \
  // UPDATE_RMP_PARAMS(RMP, metric_steepness)

  Rmp::Parameters p = std::dynamic_pointer_cast<Rmp>(local_planner_)->getParameters();
  Rmp::ControlPoints cps = std::dynamic_pointer_cast<Rmp>(local_planner_)->getControlPoints();

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

void RmpPlugin::publishVisualizations() {
  Rmp::ControlPoints control_points = std::dynamic_pointer_cast<Rmp>(local_planner_)->getControlPoints();

  int i = 0;
  visualization_msgs::MarkerArray cp_vis;
  for (auto cp : control_points) {
    // Control point marker
    {
      visualization_msgs::Marker marker;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = BasePlugin::base_frame_;
      marker.ns = "control_point";
      marker.id = i;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = cp.position(0);
      marker.pose.position.y = cp.position(1);
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 2 * cp.radius;
      marker.scale.y = 2 * cp.radius;
      marker.scale.z = 2 * cp.radius;
      marker.color.r = cp.color(0);
      marker.color.g = cp.color(1);
      marker.color.b = cp.color(2);
      marker.color.a = 0.2;
      // Add marker
      cp_vis.markers.push_back(marker);
    }

    for (auto rmp : cp.rmps) {
      std::string name = rmp.first;
      rmp::Rmp3 policy = rmp.second;

      // Accelerations
      {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = BasePlugin::base_frame_;
        marker.ns = "acc_" + name;
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = cp.position(0);
        marker.pose.position.y = cp.position(1);
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.02;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = policy.color()(0);
        marker.color.g = policy.color()(1);
        marker.color.b = policy.color()(2);
        marker.color.a = 1.0;
        geometry_msgs::Point p1;
        p1.x = 0.0;
        p1.y = 0.0;
        p1.z = 0.0;
        geometry_msgs::Point p2;
        p2.x = policy.acceleration()(0) * 0.05 * policy.weight();
        p2.y = policy.acceleration()(1) * 0.05 * policy.weight();
        p2.z = 0.0;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        // Add marker
        cp_vis.markers.push_back(marker);
      }

      // Angular accelerations
      // {
      //   visualization_msgs::Marker marker;
      //   marker.header = header;
      //   marker.header.frame_id = BasePlugin::base_frame_;
      //   marker.ns = "angular_acc_" + acc.id_;
      //   marker.id = i;
      //   marker.type = visualization_msgs::Marker::ARROW;
      //   marker.action = visualization_msgs::Marker::ADD;
      //   marker.pose.position.x = cp.position(0);
      //   marker.pose.position.y = cp.position(1);
      //   marker.pose.position.z = 0.0;
      //   marker.pose.orientation.x = 0.0;
      //   marker.pose.orientation.y = 0.0;
      //   marker.pose.orientation.z = 0.0;
      //   marker.pose.orientation.w = 1.0;
      //   marker.scale.x = 0.04;
      //   marker.scale.y = 0.07;
      //   marker.scale.z = 0.07;
      //   marker.color.r = policy.color()(0);
      //   marker.color.g = policy.color()(1);
      //   marker.color.b = policy.color()(2);
      //   marker.color.a = 1.0;
      //   geometry_msgs::Point p1;
      //   p1.x = 0.0;
      //   p1.y = 0.0;
      //   p1.z = 0.0;
      //   geometry_msgs::Point p2;
      //   p2.x = 0.0;
      //   p2.y = acc.angular_acc_ * 0.05 * policy.weight();
      //   p2.z = 0.0;
      //   marker.points.push_back(p1);
      //   marker.points.push_back(p2);
      //   // Add marker
      //   cp_vis.markers.push_back(marker);
      // }

      // Metric
      {
        visualization_msgs::Marker marker;
        // Metric
        // We first need to compute the eigenvectors and eigenvalues (based on supereight atlas)
        Eigen::Vector2d vis_eigenvalues(Eigen::Vector2d::Zero());
        Eigen::Matrix2d vis_eigenvectors(Eigen::Matrix2d::Zero());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> vis_solver(policy.metric().block(0, 0, 2, 2));

        if (vis_solver.info() == Eigen::Success) {
          vis_eigenvalues = vis_solver.eigenvalues();
          vis_eigenvectors = vis_solver.eigenvectors();
        } else {
          ROS_WARN_STREAM("Couldn't compute eigenvectors for metric of acc [" << name << "]");
          continue;
        }
        // Create rotation matrix from eigenvectors
        Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
        rot.block(0, 0, 2, 2) = vis_eigenvectors;

        // Create pose from rotation matrix and control point coords
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translate(Eigen::Vector3d(cp.position(0), cp.position(1), 0.0));
        pose.rotate(rot);
        geometry_msgs::Pose metric_pose;
        tf::poseEigenToMsg(pose, metric_pose);

        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = BasePlugin::base_frame_;
        marker.ns = "metric_" + name;
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = metric_pose;
        marker.scale.x = sqrt(vis_eigenvalues[0]) * 1.0;
        marker.scale.y = sqrt(vis_eigenvalues[1]) * 1.0;
        marker.scale.z = 0.1;
        marker.color.r = policy.color()(0);
        marker.color.g = policy.color()(1);
        marker.color.b = policy.color()(2);
        marker.color.a = 0.2;

        // Add marker
        cp_vis.markers.push_back(marker);
      }

      // Update ids
      i++;
    }
  }

  // Publish
  control_points_pub_.publish(cp_vis);
}

}  // namespace field_local_planner
