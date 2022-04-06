#pragma once
#include <locally_reactive_controller/controllers/controller_base.hpp>

class TracklineControllerParameters : public ControllerParameters {
public:
  TracklineControllerParameters() {}
};

class TracklineController : public ControllerBase {

public:
  TracklineController(const ControllerParameters& params) 
                      : ControllerBase(params){};
  
  // Set parameters
  void updateControllerParameters(ros::NodeHandle& node_handle) {}

  // Visualizations
  void setupVisualizations(ros::NodeHandle& node_handle) {}
  void publishVisualizations(const std_msgs::Header& header) {}

  // Dynamic reconfigure server
  void setupDynamicReconfigureServer(ros::NodeHandle& node_handle) {}

private:
  // Implementation of virtual ControllerBase methods
  void customPreProcessController();
  void customPreProcessLocalMap() {}
  ControllerBase::OutputAction computeCommandGoalBehind(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity);
  ControllerBase::OutputAction computeCommandTurnToGoal(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity);
  ControllerBase::OutputAction computeCommandForward(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity);
  ControllerBase::OutputAction computeCommandTurnToDestination(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity);
  ControllerBase::OutputAction computeCommandFinished(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity);
  void computeCommandPostProcess(ControllerBase::OutputAction& action, 
                                  Eigen::Vector3d& output_linear_velocity,
                                  Eigen::Vector3d& output_angular_velocity,
                                  std::vector<Eigen::Vector3d>& path_to_goal);

  // Helper to compute trackline velocity
  void computeTracklineVelocityCommand(Eigen::Isometry3d goal_pose, Eigen::Isometry3d current_pose);

private:
  TracklineControllerParameters controller_params_;

  // Trackline directed velocity commands
  double trackline_linvel_x_, trackline_linvel_y_;
  // does not command angular velocity

  // Goal directed velocity commands
  double goal_linvel_x_;
  double goal_linvel_y_;
  double goal_angvel_z_;
};
