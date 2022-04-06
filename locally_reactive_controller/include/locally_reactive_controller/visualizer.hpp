#pragma once
#include <deque>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/console.h>
#include <std_msgs/Header.h>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <locally_reactive_controller/utils/path.hpp>
#include <pcl/features/normal_3d.h>
#include <visualization_msgs/MarkerArray.h>

#include <locally_reactive_controller/controllers/controller_base.hpp>

class Visualizer {

public:
  // Constructor
  Visualizer();
  
  // Read ROS parameters and prepare subscribers/publishers
  void setup(ros::NodeHandle& node_handle);

  // Publish visualizations
  void publishCurrentGoal(const Eigen::Isometry3d& pose, const std_msgs::Header& header);
  
  void publishRemainingGoals(const std::deque<Eigen::Isometry3d>& remaining_goals, const std_msgs::Header& header);
  
  void publishStartingGoal(const Eigen::Isometry3d& pose, const std_msgs::Header& header);

  void publishEstimatedVelocity(const Eigen::Vector3d& linear_velocity, 
                              const Eigen::Vector3d& angular_velocity,
                              const std_msgs::Header& header);
  
  void publishVelocityCombined(const Eigen::Vector3d& linear_velocity, 
                              const Eigen::Vector3d& angular_velocity,
                              const std_msgs::Header& header);

  void publishVelocityTrackLine(const double& trackline_linear_velocity_x,
                                const double& trackline_linear_velocity_y,
                                const std_msgs::Header& header);
  void publishVelocityGoal(const double& goal_linear_velocity_x,
                            const double& goal_linear_velocity_y,
                            const double& goal_angular_velocity_z,
                            const std_msgs::Header& header);
  void publishReferencePath(const std::vector<Eigen::Vector3d>& path,
                            const std_msgs::Header& header);

  void publishCollisionBox(float robot_length,
                           float robot_width,
                           float robot_height,
                           float clearance,
                           bool colliding,
                           const std_msgs::Header& header);



private:

  // Visualizations
  ros::Publisher current_goal_pub_;
  ros::Publisher remaining_goals_pub_;
  ros::Publisher starting_goal_pub_;
  ros::Publisher velocity_estimated_pub_;
  ros::Publisher velocity_estimated_twist_pub_;
  ros::Publisher velocity_combined_pub_;
  ros::Publisher velocity_trackline_pub_;
  ros::Publisher velocity_goal_pub_;
  ros::Publisher reference_path_pub_; // former marker pub
  ros::Publisher collision_box_pub_;
};
