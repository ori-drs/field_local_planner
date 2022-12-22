#pragma once
#include <field_local_planners/base_local_planner.hpp>

namespace field_local_planners {

class Trackline : public BaseLocalPlanner {
 public:
  struct Parameters : BaseLocalPlanner::Parameters {
    double angular_gain_p;
    double linear_gain_p;
  };

  // For the internal state machine
  enum class State {
    TURN_TO_GOAL = 0,         // First the robot will rotate to the goal
    FORWARD = 1,              // Move forward tracking a line
    TURN_TO_DESTINATION = 2,  // Rotate to match the final destination
    UNKNOWN = -1,
  };

 public:
  Trackline();

  Twist computeTwist();

 protected:
  Parameters parameters_;

  State state_;

  // Trackline directed velocity commands
  double trackline_linvel_x_, trackline_linvel_y_;

  // Goal directed velocity commands
  double goal_linvel_x_;
  double goal_linvel_y_;
  double goal_angvel_z_;
};

}  // namespace field_local_planners

//   // Set parameters
//   void updateControllerParameters(ros::NodeHandle& node_handle) {}

//   // Visualizations
//   void setupVisualizations(ros::NodeHandle& node_handle) {}
//   void publishVisualizations(const std_msgs::Header& header) {}

//   // Dynamic reconfigure server
//   void setupDynamicReconfigureServer(ros::NodeHandle& node_handle) {}

// private:
//   // Implementation of virtual ControllerBase methods
//   void customPreProcessController();
//   void customPreProcessLocalMap() {}
//   ControllerBase::OutputAction computeCommandGoalBehind(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d&
//   output_angular_velocity); ControllerBase::OutputAction computeCommandTurnToGoal(Eigen::Vector3d& output_linear_velocity,
//   Eigen::Vector3d& output_angular_velocity); ControllerBase::OutputAction computeCommandForward(Eigen::Vector3d& output_linear_velocity,
//   Eigen::Vector3d& output_angular_velocity); ControllerBase::OutputAction computeCommandTurnToDestination(Eigen::Vector3d&
//   output_linear_velocity, Eigen::Vector3d& output_angular_velocity); ControllerBase::OutputAction computeCommandFinished(Eigen::Vector3d&
//   output_linear_velocity, Eigen::Vector3d& output_angular_velocity); void computeCommandPostProcess(ControllerBase::OutputAction& action,
//                                   Eigen::Vector3d& output_linear_velocity,
//                                   Eigen::Vector3d& output_angular_velocity,
//                                   std::vector<Eigen::Vector3d>& path_to_goal);

//   // Helper to compute trackline velocity
//   void computeTracklineVelocityCommand(Eigen::Isometry3d goal_pose, Eigen::Isometry3d current_pose);

// private:
//   TracklinePlannerParameters controller_params_;

//   // Trackline directed velocity commands
//   double trackline_linvel_x_, trackline_linvel_y_;
//   // does not command angular velocity

//   // Goal directed velocity commands
//   double goal_linvel_x_;
//   double goal_linvel_y_;
//   double goal_angvel_z_;