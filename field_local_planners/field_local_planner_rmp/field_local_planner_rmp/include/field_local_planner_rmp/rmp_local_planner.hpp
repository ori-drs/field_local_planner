#pragma once
#include <field_local_planner_base/local_planner.hpp>
#include <field_local_planner_base/utils.hpp>

// RMP stuff
#include <gtsam/base/Vector.h>
#include <rmp/rmp.hpp>

#include <map>

namespace field_local_planner {

class Rmp : public BaseLocalPlanner {
 public:

  struct RmpParameters {
    std::string name;
    double weight;
    double gain;
    Vector3 color;
    std::string metric_type;
    double metric_offset;
    double metric_steepness;
  };

  struct Parameters : BaseLocalPlanner::Parameters {
    double robot_clearance = 0.1;
    double sphere_radius_factor = 1.0;
    double integration_time = 1.0;
    std::map<std::string, RmpParameters> rmp_parameters;
  };

  struct RmpViz {
    std::string name;
    Vector2 acc;
    Matrix2 metric;
    Vector3 color;
    double weight;
  };

  struct ControlPoint {
    std::string id;
    std::string frame_id;
    double radius;
    Vector2 position;
    Vector3 color;
    std::vector<std::string> affected_by;
    std::map<std::string, RmpViz> rmp_viz;
  };

 public:
  Rmp();
  void loadParameters(const Parameters& parameters);
  void addControlPoint(const ControlPoint& cp);

  Twist computeTwist();
  Path computePath();

  // Utils
  std::vector<std::string> getAvailableRmps();

 private:
  void computeOptimalAcceleration();
  rmp::Rmp3 makeGeodesicGoalPolicy(ControlPoint& cp);
  rmp::Rmp3 makeGeodesicHeadingPolicy(ControlPoint& cp);
  rmp::Rmp3 makeGoalPositionPolicy(ControlPoint& cp);
  rmp::Rmp3 makeGoalOrientationPolicy(ControlPoint& cp);
  rmp::Rmp3 makeVelocityHeadingPolicy(ControlPoint& cp);
  rmp::Rmp3 makeDampingPolicy(ControlPoint& cp);
  rmp::Rmp2 makeSdfObstaclePolicy(ControlPoint& cp);
  rmp::Rmp3 makeRegularizationPolicy(ControlPoint& cp);
  void getGradientsFromGridMap(const std::string& layer, const Pose2& T_m_b_query, double& distance, Vector2& grad_in_base);

 protected:
  Parameters parameters_;

  // Other internal variables
  // Internal
  Vector3 velocity_2d_;
  Vector3 optimal_velocity_;
  Vector3 optimal_acc_;
  Matrix3 optimal_metric_;

  Pose3 T_m_b_;  // Base in map frame

  Pose2 T_m_b_SE2_;  // Pose of robot in map frame
  Pose2 T_f_b_SE2_;  // Pose of robot in fixed frame
  Pose2 T_f_g_SE2_;  // Pose of goal in fixed frame

  // Control points stuff
  std::vector<ControlPoint> control_points_;
  // std::vector<AccelerationVisualization> vis_accelerations_;
};

}  // namespace field_local_planner
