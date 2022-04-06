#include <locally_reactive_controller/visualizer.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <locally_reactive_controller/utils/path.hpp>
#include <eigen_conversions/eigen_msg.h>

 // Constructor
Visualizer::Visualizer() {

}
  
// Read ROS parameters and prepare subscribers/publishers
void Visualizer::setup(ros::NodeHandle& node_handle){

  // Diagnostics
  current_goal_pub_       = node_handle.advertise<geometry_msgs::PoseStamped>("/locally_reactive_controller/current_goal", 10);
  remaining_goals_pub_    = node_handle.advertise<geometry_msgs::PoseArray>("/locally_reactive_controller/remaining_goals", 10);
  starting_goal_pub_      = node_handle.advertise<geometry_msgs::PoseStamped>("/locally_reactive_controller/starting_pose", 10);
  velocity_estimated_pub_  = node_handle.advertise<geometry_msgs::WrenchStamped>("/locally_reactive_controller/estimated_velocity", 10);
  velocity_estimated_twist_pub_  = node_handle.advertise<geometry_msgs::TwistStamped>("/locally_reactive_controller/estimated_twist", 10);

  velocity_combined_pub_  = node_handle.advertise<geometry_msgs::WrenchStamped>("/locally_reactive_controller/velocity_combined", 10);
  velocity_trackline_pub_ = node_handle.advertise<geometry_msgs::WrenchStamped>("/locally_reactive_controller/velocity_goal", 10);
  velocity_goal_pub_      = node_handle.advertise<geometry_msgs::WrenchStamped>("/locally_reactive_controller/velocity_trackline", 10);
  reference_path_pub_     = node_handle.advertise<nav_msgs::Path>("/locally_reactive_controller/reference_path", 10);
  collision_box_pub_      = node_handle.advertise<visualization_msgs::Marker>("/locally_reactive_controller/collision_box", 10);
}


void Visualizer::publishCurrentGoal(const Eigen::Isometry3d& pose, const std_msgs::Header& header){
  // Visualize the current goal
  geometry_msgs::PoseStamped msg;
  msg.header = header;
  tf::poseEigenToMsg (pose, msg.pose);

  // Publish
  current_goal_pub_.publish(msg);
}

void Visualizer::publishRemainingGoals(const std::deque<Eigen::Isometry3d>& remaining_goals, const std_msgs::Header& header){
  if (remaining_goals.size() <= 0){
    return;
  }
  
  //std::cout << "got " << remainingGoals.size() << " old poses\n";
  geometry_msgs::PoseArray msg_array;
  msg_array.header = header;
  // msg_array.header.frame_id = remaining_goals[0].frame_id; // assumes all frame_id are the same

  for (size_t i=0 ; i < remaining_goals.size(); i++){
    geometry_msgs::Pose p;
    tf::poseEigenToMsg ( remaining_goals[i], p);
    msg_array.poses.push_back( p );
  }

  // Publish
  remaining_goals_pub_.publish(msg_array);
}

void Visualizer::publishStartingGoal(const Eigen::Isometry3d& pose, const std_msgs::Header& header){
  
  geometry_msgs::PoseStamped msg;
  msg.header = header;
  tf::poseEigenToMsg (pose, msg.pose);

  // Publish
  starting_goal_pub_.publish(msg);
}

void Visualizer::publishEstimatedVelocity(const Eigen::Vector3d& linear_velocity, 
                                         const Eigen::Vector3d& angular_velocity,
                                         const std_msgs::Header& header) {
  geometry_msgs::WrenchStamped ws;
  ws.header = header;
  ws.wrench.force.x = linear_velocity[0];
  ws.wrench.force.y = linear_velocity[1];
  ws.wrench.force.z = linear_velocity[2];
  ws.wrench.torque.x = angular_velocity[0];
  ws.wrench.torque.y = angular_velocity[1];
  ws.wrench.torque.z = fabs(angular_velocity[2]); // for some reason rviz wont visualise this if negative

  // Publish
  velocity_estimated_pub_.publish(ws);

  geometry_msgs::TwistStamped ts;
  ts.header = header;
  ts.twist.linear.x = linear_velocity[0];
  ts.twist.linear.y = linear_velocity[1];
  ts.twist.linear.z = linear_velocity[2];
  ts.twist.angular.x = angular_velocity[0];
  ts.twist.angular.y = angular_velocity[1];
  ts.twist.angular.z = angular_velocity[2];

  // Publish
  velocity_estimated_twist_pub_.publish(ts);
}

void Visualizer::publishVelocityCombined(const Eigen::Vector3d& linear_velocity, 
                                         const Eigen::Vector3d& angular_velocity,
                                         const std_msgs::Header& header) {
  geometry_msgs::WrenchStamped ws;
  ws.header = header;
  ws.wrench.force.x = linear_velocity[0];
  ws.wrench.force.y = linear_velocity[1];
  ws.wrench.force.z = linear_velocity[2];
  ws.wrench.torque.x = angular_velocity[0];
  ws.wrench.torque.y = angular_velocity[1];
  ws.wrench.torque.z = fabs(angular_velocity[2]); // for some reason rviz wont visualise this if negative

  // Publish
  velocity_combined_pub_.publish(ws);
}

void Visualizer::publishVelocityTrackLine(const double& trackline_linear_velocity_x,
                                          const double& trackline_linear_velocity_y,
                                          const std_msgs::Header& header){
  geometry_msgs::WrenchStamped ws;
  ws.header = header;
  ws.wrench.force.x = trackline_linear_velocity_x;
  ws.wrench.force.y = trackline_linear_velocity_y;
  velocity_trackline_pub_.publish(ws);
}

void Visualizer::publishVelocityGoal(const double& goal_linear_velocity_x,
                                     const double& goal_linear_velocity_y,
                                     const double& goal_angular_velocity_z,
                                     const std_msgs::Header& header){
  geometry_msgs::WrenchStamped ws;
  ws.header = header;
  ws.wrench.force.x = goal_linear_velocity_x;
  ws.wrench.force.y = goal_linear_velocity_y;
  ws.wrench.torque.z = fabs(goal_angular_velocity_z); // for some reason rviz wont visualise this if nexative
  velocity_goal_pub_.publish(ws);
}

void Visualizer::publishReferencePath(const std::vector<Eigen::Vector3d>& path,
                                      const std_msgs::Header& header){
  
  if(path.empty())
    return;

  nav_msgs::Path path_msg;
  path_msg.header = header;
  path_msg.poses.resize(path.size());

  for(size_t i = 0; i<path.size(); i++){
    path_msg.poses[i].pose.position.x = path.at(i)(0);
    path_msg.poses[i].pose.position.y = path.at(i)(1);
    path_msg.poses[i].pose.position.z = path.at(i)(2);
  }

  // Publish
  reference_path_pub_.publish(path_msg);
}

void Visualizer::publishCollisionBox(float robot_length,
                                    float robot_width,
                                    float robot_height,
                                    float clearance,
                                    bool colliding,
                                    const std_msgs::Header& header) {
  visualization_msgs::Marker marker;
  marker.header = header;
  marker.ns = "collision_box";
  marker.id = 0;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = robot_length + clearance;
  marker.scale.y = robot_width  + clearance;
  marker.scale.z = robot_height + clearance;

  if(colliding){
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.7;
  } else{
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.7;
  }

  collision_box_pub_.publish(marker);
}
