#include <locally_reactive_controller/controllers/rmp_controller.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace gtsam;

//-------------------------------------------------------------------------------------------------
// Constructor and parameters
//-------------------------------------------------------------------------------------------------
RmpController::RmpController(const ControllerParameters& params) : ControllerBase(params)
{
    node_handle_ = ros::NodeHandle("~rmp");

    // Update parameters
    ROS_WARN("[ControllerBase] Setting up Controller Parameters");
    updateControllerParameters(node_handle_);

    // Setup Dynamic reconfigure
    ROS_WARN("[ControllerBase] Setting up Dynamic reconfigure");
    setupDynamicReconfigureServer(node_handle_);

    // Setup visualizations
    ROS_WARN("[ControllerBase] Setting up Visualizations");
    setupVisualizations(node_handle_);
};

void RmpController::updateControllerParameters(ros::NodeHandle& node_handle)
{
    controller_params_.config_folder_ =
        ros::package::getPath("locally_reactive_controller") + "/config/controllers/rmp/";

    // RMP params
    controller_params_.sphere_radius_factor_ =
        utils::getParameterDefault<double>(node_handle, "sphere_radius_factor", 1.0);
    controller_params_.geodesic_goal_gain_ =
        utils::getParameterDefault<double>(node_handle, "geodesic_goal_gain", 7.0);
    controller_params_.geodesic_goal_offset_ =
        utils::getParameterDefault<double>(node_handle, "geodesic_goal_offset", 0.5);
    controller_params_.geodesic_goal_steepness_ =
        utils::getParameterDefault<double>(node_handle, "geodesic_goal_steepness", 1.0);
    controller_params_.euclidean_goal_gain_ =
        utils::getParameterDefault<double>(node_handle, "euclidean_goal_gain", 7.0);
    controller_params_.euclidean_goal_offset_ =
        utils::getParameterDefault<double>(node_handle, "euclidean_goal_offset", 0.5);
    controller_params_.euclidean_goal_steepness_ =
        utils::getParameterDefault<double>(node_handle, "euclidean_goal_steepness", 1.0);
    controller_params_.damping_ = utils::getParameterDefault<double>(node_handle, "damping", 1.0);
    controller_params_.obstacle_gain_ =
        utils::getParameterDefault<double>(node_handle, "obstacle_gain", 1.0);
    controller_params_.obstacle_offset_ =
        utils::getParameterDefault<double>(node_handle, "obstacle_offset", 0.1);
    controller_params_.obstacle_steepness_ =
        utils::getParameterDefault<double>(node_handle, "obstacle_steepness", 1.0);
    controller_params_.heading_gain_ =
        utils::getParameterDefault<double>(node_handle, "heading_gain", 1.0);
    controller_params_.heading_offset_ =
        utils::getParameterDefault<double>(node_handle, "heading_offset", 0.5);
    controller_params_.heading_steepness_ =
        utils::getParameterDefault<double>(node_handle, "heading_steepness", 1.0);
    controller_params_.regularization_ =
        utils::getParameterDefault<double>(node_handle, "acceleration_regularization", 0.1);
    controller_params_.integration_time_ =
        utils::getParameterDefault<double>(node_handle, "integration_time", 0.3);

    controller_params_.geodesic_goal_weight_ =
        utils::getParameterDefault<double>(node_handle, "geodesic_goal_weight", 1.0);
    controller_params_.euclidean_goal_weight_ =
        utils::getParameterDefault<double>(node_handle, "euclidean_goal_weight", 1.0);
    controller_params_.damping_weight_ =
        utils::getParameterDefault<double>(node_handle, "damping_weight", 1.0);
    controller_params_.obstacle_weight_ =
        utils::getParameterDefault<double>(node_handle, "obstacle_weight", 1.0);
    controller_params_.heading_weight_ =
        utils::getParameterDefault<double>(node_handle, "heading_weight", 1.0);
    controller_params_.check_internal_collisions_ =
        utils::getParameterDefault<bool>(node_handle, "check_internal_collisions", false);

    // Prepare control points
    prepareControlPoints();
}

void RmpController::setupDynamicReconfigureServer(ros::NodeHandle& node_handle)
{
    // Create Dynamic Reconfigure server passing correct node handle
    dynamic_reconfigure_server_ = std::make_shared<
        dynamic_reconfigure::Server<locally_reactive_controller::RmpControllerConfig>>(node_handle);

    dynamic_reconfigure_callback_ =
        boost::bind(&RmpController::dynamicReconfigureCallback, this, _1, _2);
    dynamic_reconfigure_server_->setCallback(dynamic_reconfigure_callback_);
}

void RmpController::dynamicReconfigureCallback(
    locally_reactive_controller::RmpControllerConfig& config, uint32_t level)
{
    // RMP parameters
    utils::assignAndPrintDiff("sphere_radius_factor", controller_params_.sphere_radius_factor_,
                              config.sphere_radius_factor);
    utils::assignAndPrintDiff("goal_gain", controller_params_.geodesic_goal_gain_,
                              config.geodesic_goal_gain);
    utils::assignAndPrintDiff("goal_offset", controller_params_.geodesic_goal_offset_,
                              config.geodesic_goal_offset);
    utils::assignAndPrintDiff("goal_steepness", controller_params_.geodesic_goal_steepness_,
                              config.geodesic_goal_steepness);
    utils::assignAndPrintDiff("euclidean_goal_gain", controller_params_.euclidean_goal_gain_,
                              config.euclidean_goal_gain);
    utils::assignAndPrintDiff("euclidean_goal_offset", controller_params_.euclidean_goal_offset_,
                              config.euclidean_goal_offset);
    utils::assignAndPrintDiff("euclidean_goal_steepness",
                              controller_params_.euclidean_goal_steepness_,
                              config.euclidean_goal_steepness);
    utils::assignAndPrintDiff("damping", controller_params_.damping_, config.damping);
    utils::assignAndPrintDiff("obstacle_gain", controller_params_.obstacle_gain_,
                              config.obstacle_gain);
    utils::assignAndPrintDiff("obstacle_offset", controller_params_.obstacle_offset_,
                              config.obstacle_offset);
    utils::assignAndPrintDiff("obstacle_steepness", controller_params_.obstacle_steepness_,
                              config.obstacle_steepness);
    utils::assignAndPrintDiff("heading_gain", controller_params_.heading_gain_,
                              config.heading_gain);
    utils::assignAndPrintDiff("heading_offset", controller_params_.heading_offset_,
                              config.heading_offset);
    utils::assignAndPrintDiff("heading_steepness", controller_params_.heading_steepness_,
                              config.heading_steepness);
    utils::assignAndPrintDiff("acceleration_regularization", controller_params_.regularization_,
                              config.acceleration_regularization);
    utils::assignAndPrintDiff("integration_time", controller_params_.integration_time_,
                              config.integration_time);

    utils::assignAndPrintDiff("geodesic_goal_weight", controller_params_.geodesic_goal_weight_,
                              config.geodesic_goal_weight);
    utils::assignAndPrintDiff("euclidean_goal_weight", controller_params_.euclidean_goal_weight_,
                              config.euclidean_goal_weight);
    utils::assignAndPrintDiff("damping_weight", controller_params_.damping_weight_,
                              config.damping_weight);
    utils::assignAndPrintDiff("obstacle_weight", controller_params_.obstacle_weight_,
                              config.obstacle_weight);
    utils::assignAndPrintDiff("heading_weight", controller_params_.heading_weight_,
                              config.heading_weight);
    utils::assignAndPrintDiff("check_internal_collisions",
                              controller_params_.check_internal_collisions_,
                              config.check_internal_collisions);

    // Prepare control points
    prepareControlPoints();
}

//-------------------------------------------------------------------------------------------------
// Control points
//-------------------------------------------------------------------------------------------------
void RmpController::prepareControlPoints()
{
    control_points_.clear();

    // Open YAML file
    ROS_INFO_STREAM("Opening file: " << controller_params_.config_folder_ + "control_points.yaml");
    YAML::Node cp_config =
        YAML::LoadFile(controller_params_.config_folder_ + "control_points.yaml");

    // Preallocate robot dimensions
    double robot_half_length = (params_.robot_length_ + params_.robot_clearance_) / 2;
    double robot_half_width = (params_.robot_width_ + params_.robot_clearance_) / 2;

    Eigen::Rotation2Dd flip_front_direction(0);
    if (params_.back_is_front_)
    {
        flip_front_direction = Eigen::Rotation2Dd(M_PI);
    }

    // Read the YAML file
    for (YAML::const_iterator it = cp_config["control_points"].begin();
         it != cp_config["control_points"].end(); ++it)
    {
        ControlPointStruct control_point;
        // Id
        control_point.id_ = it->first.as<std::string>();
        ROS_INFO_STREAM("   Filling control point " << control_point.id_);

        YAML::Node cp_attributes = it->second.as<YAML::Node>();
        // Point
        std::vector<double> factors =
            utils::get<std::vector<double>>(cp_attributes, "point_factor", std::vector<double>());
        control_point.point_ =
            flip_front_direction *
            Eigen::Vector2d(robot_half_length * factors[0], robot_half_width * factors[1]);
        ROS_INFO_STREAM("   point " << control_point.point_.transpose());
        // Color
        std::vector<double> rgb =
            utils::get<std::vector<double>>(cp_attributes, "color", std::vector<double>());
        control_point.color_ = Eigen::Vector3f(rgb[0], rgb[1], rgb[2]);
        // Accelerations
        control_point.acc_ = Eigen::Vector2d::Zero();
        // Metric
        control_point.metric_ = Eigen::Matrix2d::Identity();
        // Affected by forces
        control_point.affected_by_goal_ = utils::get<bool>(cp_attributes, "affected_by_goal", true);
        control_point.affected_by_obstacle_ =
            utils::get<bool>(cp_attributes, "affected_by_obstacle", true);
        // Radius
        control_point.radius_ =
            std::min(params_.robot_length_, params_.robot_width_) * 0.5 *
            controller_params_
                .sphere_radius_factor_;  // utils::get<float>(cp_attributes, "radius", 0.1);

        // Add to control points
        control_points_.push_back(control_point);
    }
}

//-------------------------------------------------------------------------------------------------
// Controller methods
//-------------------------------------------------------------------------------------------------
void RmpController::customPreProcessLocalMap() {}

//-------------------------------------------------------------------------------------------------
// Controller methods
//-------------------------------------------------------------------------------------------------
void RmpController::customPreProcessController()
{
    // Reset flags
    is_colliding_ = false;

    // Fill robot velocity
    velocity_2d_.x() = b_v_b_t_(3);  // x component
    velocity_2d_.y() = b_v_b_t_(4);  // y component
    velocity_2d_.z() = b_v_b_t_(2);  // angular z component

    ROS_INFO_STREAM_THROTTLE(1, "last optimal acc: " << optimal_acc_.transpose());
    ROS_INFO_STREAM_THROTTLE(1, "last optimal vel: " << optimal_velocity_.transpose());
    ROS_INFO_STREAM_THROTTLE(1, "Current velocity: " << velocity_2d_.transpose());

    // Compute pose in local map frame, since it's used to query the grid map
    std::string map_frame = grid_map_.getFrameId();
    tf::StampedTransform fixed_to_map_transform;
    Eigen::Isometry3d fixed_to_map;
    try
    {
        // Listen transform
        tf_listener_.lookupTransform(map_frame, params_.fixed_frame_, ros::Time(0),
                                     fixed_to_map_transform);
        // Convert to Isometry3d
        fixed_to_map = Eigen::Isometry3d::Identity();
        tf::transformTFToEigen(fixed_to_map_transform, fixed_to_map);

        // Update current pose
        current_pose_in_map_ = fixed_to_map * f_T_fb_t_;
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR_STREAM("tf listener failed to obtain pose T_" << map_frame << "_"
                                                                << params_.fixed_frame_);
        ROS_ERROR("%s", ex.what());
        return;
    }

    // Create gtsam poses
    gtsam_current_pose_in_fixed_ =
        gtsam::Pose2(f_T_fb_t_2d_.translation().x(), f_T_fb_t_2d_.translation().y(),
                     Eigen::Rotation2Dd(f_T_fb_t_2d_.linear()).angle());

    Eigen::AngleAxisd axis_angle_pose_in_map(current_pose_in_map_.rotation());
    double pose_in_map_angle = axis_angle_pose_in_map.axis()(2) * axis_angle_pose_in_map.angle();
    gtsam_current_pose_in_map_ =
        gtsam::Pose2(current_pose_in_map_.translation().x(), current_pose_in_map_.translation().y(),
                     pose_in_map_angle);

    Eigen::AngleAxisd axis_angle(f_Tgoal_fb_t_.rotation());
    double goal_angle = axis_angle.axis()(2) * axis_angle.angle();
    gtsam_current_goal_in_fixed_ =
        gtsam::Pose2(f_Tgoal_fb_t_.translation().x(), f_Tgoal_fb_t_.translation().y(), goal_angle);

    // Compute optimal acceleration
    profiler_ptr_->startEvent("1.3.compute_rmp_acc");
    computeOptimalAcceleration();
    profiler_ptr_->endEvent("1.3.compute_rmp_acc");
}

ControllerBase::OutputAction RmpController::computeCommandGoalBehind(
    Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity)
{
    // Compute angular velocity command
    output_linear_velocity(0) = 0.0;
    output_linear_velocity(1) = 0.0;
    output_linear_velocity(2) = 0.0;

    // compute angular velocity only command
    output_angular_velocity(0) = 0.0;
    output_angular_velocity(1) = 0.0;
    output_angular_velocity(2) = 0.0;

    return ControllerBase::OutputAction::SEND_COMMAND;
}

ControllerBase::OutputAction RmpController::computeCommandTurnToGoal(
    Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity)
{
    return computeCommandForward(output_linear_velocity, output_angular_velocity);
}

ControllerBase::OutputAction RmpController::computeCommandForward(
    Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity)
{
    output_linear_velocity(0) = optimal_velocity_.x();
    output_linear_velocity(1) = optimal_velocity_.y();
    output_angular_velocity(2) = optimal_velocity_.z();

    return ControllerBase::OutputAction::SEND_COMMAND;
}

ControllerBase::OutputAction RmpController::computeCommandTurnToDestination(
    Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity)
{
    return computeCommandForward(output_linear_velocity, output_angular_velocity);
}

ControllerBase::OutputAction RmpController::computeCommandFinished(
    Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity)
{
    ControllerBase::OutputAction action = ControllerBase::OutputAction::SEND_COMMAND;

    if (goals_.size() > 0)
    {
        // ROS_INFO("Continuing to walk without stopping");
        action = SEND_NOTHING;
    }

    velocity_2d_ = gtsam::Vector3::Zero();

    return action;
}

void RmpController::computeCommandPostProcess(ControllerBase::OutputAction& action,
                                              Eigen::Vector3d& output_linear_velocity,
                                              Eigen::Vector3d& output_angular_velocity,
                                              std::vector<Eigen::Vector3d>& path_to_goal)
{
    path_to_goal.clear();

    // Integrate the velocity to get a path
    const double T = 2;  // seconds
    for (double dt = 0; dt < T; dt += 0.1)
    {
        gtsam::Pose2 pose_t = gtsam::Pose2::Expmap(optimal_velocity_ * dt);

        path_to_goal.push_back(Eigen::Vector3d(pose_t.x(), pose_t.y(), 0.0));
    }
}

//-------------------------------------------------------------------------------------------------
// RMP optimization
//-------------------------------------------------------------------------------------------------

void RmpController::computeOptimalAcceleration()
{
    ROS_INFO_STREAM_THROTTLE(1, "---compute optimal acceleration---");

    // ------------------------------------------------------------------------------------
    // Initialize
    // ------------------------------------------------------------------------------------

    // Create problem
    gtsam::Symbol acc_se2_key = gtsam::symbol_shorthand::X(0);
    gtsam::Symbol acc_diff_key = gtsam::symbol_shorthand::X(1);
    Vector3_ acc_se2(acc_se2_key);    // Full SE(2) acceleration
    Vector2_ acc_diff(acc_diff_key);  // Differentially constrained acceleration

    // Create general RMP problem
    RmpProblem problem;

    // Create initial values
    // We use the last optimal acceleration
    Values initial_se2_acc;
    initial_se2_acc.insert(acc_se2_key, optimal_acc_);

    Values initial_diff_acc;
    initial_diff_acc.insert(acc_diff_key, Vector2(optimal_acc_(0), optimal_acc_(2)));

    // Create differential model (if required)
    DifferentialModel motion_model(0.0);
    DifferentialModel_ motion_model_(motion_model);

    // Build control points RMPs
    profiler_ptr_->startEvent("1.3.1.prepare_rmps");

    // Clear visualizations
    vis_accelerations_.clear();
    Eigen::Rotation2Dd flip_front_direction(0);
    double flip_front_angle = 1.0;
    if (params_.back_is_front_)
    {
        flip_front_direction = Eigen::Rotation2Dd(M_PI);
        flip_front_angle = -1;
    }

    // ------------------------------------------------------------------------------------
    // Add RMPs for control points
    // ------------------------------------------------------------------------------------
    for (size_t i = 0; i < control_points_.size(); i++)
    {
        // Create expressions for control points
        ControlPoint2 cp(Pose2(), control_points_.at(i).point_);
        ControlPoint2_ cp_(cp);

        // Store coordinates in fixed frame
        Vector2 cp_in_map_frame = gtsam_current_pose_in_map_ * cp.point();
        control_points_.at(i).point_fixed_ = cp_in_map_frame;
        float radius = control_points_.at(i).radius_;

        // Obstacles
        {
            // Get distance to obstacle and gradient in fixed frame
            double obstacle_distance, obstacle_grad_x, obstacle_grad_y;
            try
            {
                obstacle_distance = grid_map_.atPosition("sdf", cp_in_map_frame) - radius;
                obstacle_grad_x = grid_map_.atPosition("sdf_gradient_x", cp_in_map_frame);
                obstacle_grad_y = grid_map_.atPosition("sdf_gradient_y", cp_in_map_frame);
            }
            catch (const std::out_of_range& oor)
            {
                ROS_ERROR_STREAM("Error while trying to access [sdf] layer at "
                                 << cp.point().transpose() << "\n Error: \n"
                                 << oor.what());
            }
            Vector2 gradient_in_base_frame = gtsam_current_pose_in_map_.rotation().inverse() *
                                             Vector2(obstacle_grad_x, obstacle_grad_y);

            // Create Obstacle RMP
            SdfObstacle2Rmp obstacle_rmp(
                cp.point(), cp.apply(velocity_2d_), obstacle_distance, gradient_in_base_frame,
                controller_params_.obstacle_gain_, controller_params_.obstacle_offset_,
                controller_params_.obstacle_steepness_, controller_params_.obstacle_weight_);
            AccelerationVisualization acc;
            acc.id_ = "obstacle_" + control_points_.at(i).id_;
            acc.acc_ = flip_front_direction * obstacle_rmp.acceleration().head<2>();
            acc.metric_ = obstacle_rmp.metric().block(0, 0, 2, 2);
            acc.point_ = flip_front_direction * control_points_.at(i).point_;
            acc.color_ = Eigen::Vector3f(1.0, 0.0, 0.0);
            acc.weight_ = controller_params_.obstacle_weight_;
            vis_accelerations_.push_back(acc);
            // ROS_WARN_STREAM("[Obstacle RMP] " <<
            //                 "\n  distance:    " << obstacle_distance <<
            //                 "\n  grad_x  :    " << obstacle_grad_x <<
            //                 "\n  grad_y  :    " << obstacle_grad_y <<
            //                 "\n  acc:         " << acc.acc_.transpose() <<
            //                 // "\n  angular_acc: " << acc.angular_acc_ <<
            //                 "\n  metric: " << acc.metric_ <<
            //                 std::endl;
            //                 );

            // Add RMP
            if (control_points_.at(i).affected_by_obstacle_)
            {
                if (params_.differential_drive_)
                {
                    problem.addRmp(
                        applyControlPoint2(cp_, applyDifferentialModel(motion_model_, acc_diff)),
                        obstacle_rmp);
                }
                else
                {
                    problem.addRmp(applyControlPoint2(cp_, acc_se2), obstacle_rmp);
                }
            }
        }  // end obstacles

        // Geodesic Goal
        {
            // Get distance to goal and gradient in fixed frame
            double goal_distance, goal_grad_x, goal_grad_y;
            gtsam::Vector2 base_position = gtsam_current_pose_in_map_.translation();
            try
            {
                goal_distance = grid_map_.atPosition("geodesic", cp_in_map_frame) - radius;
                goal_grad_x = grid_map_.atPosition("geodesic_gradient_x", cp_in_map_frame);
                goal_grad_y = grid_map_.atPosition("geodesic_gradient_y", cp_in_map_frame);
            }
            catch (const std::out_of_range& oor)
            {
                ROS_ERROR_STREAM("Error while trying to access [geodesic] layer at "
                                 << cp.point().transpose() << "\n Error: \n"
                                 << oor.what());
            }
            // Create Goal RMP
            Vector2 gradient_in_base_frame =
                gtsam_current_pose_in_map_.rotation().inverse() * Vector2(goal_grad_x, goal_grad_y);

            GeodesicGoal2Rmp goal_rmp(cp.point(), cp.apply(velocity_2d_), goal_distance,
                                     gradient_in_base_frame, controller_params_.geodesic_goal_gain_,
                                     controller_params_.geodesic_goal_offset_,
                                     controller_params_.geodesic_goal_steepness_,
                                     controller_params_.geodesic_goal_weight_);
            AccelerationVisualization acc;
            acc.id_ = "geodesic_goal_"+ control_points_.at(i).id_;
            acc.acc_ = flip_front_direction * goal_rmp.acceleration().head<2>();
            acc.metric_ = goal_rmp.metric().block(0, 0, 2, 2);
            acc.point_ = flip_front_direction * control_points_.at(i).point_;
            acc.color_ = Eigen::Vector3f(1.0, 1.0, 0.0);
            acc.weight_ = controller_params_.geodesic_goal_weight_;
            vis_accelerations_.push_back(acc);

            // Add RMP
            if (control_points_.at(i).affected_by_goal_)
            {
                if (params_.differential_drive_)
                {
                    problem.addRmp(
                        applyControlPoint2(cp_, applyDifferentialModel(motion_model_, acc_diff)),
                        goal_rmp);
                }
                else
                {
                    problem.addRmp(applyControlPoint2(cp_, acc_se2), goal_rmp);
                }
            }
        }
    }

    // ------------------------------------------------------------------------------------
    // Add other RMPs to general problem
    // ------------------------------------------------------------------------------------
    // Damping RMP
    DampingRmp damping_rmp(gtsam_current_pose_in_fixed_, velocity_2d_, controller_params_.damping_,
                           controller_params_.damping_weight_);
    AccelerationVisualization acc_damping;
    acc_damping.id_ = "damping";
    acc_damping.acc_ = flip_front_direction * damping_rmp.acceleration().head<2>();
    acc_damping.angular_acc_ = flip_front_angle * damping_rmp.acceleration()(2);
    acc_damping.metric_ = damping_rmp.metric().block(0, 0, 2, 2);
    acc_damping.point_ = Eigen::Vector2d(0.0, 0.0);
    acc_damping.color_ = Eigen::Vector3f(0.0, 0.0, 1.0);
    acc_damping.weight_ = controller_params_.damping_weight_;
    vis_accelerations_.push_back(acc_damping);

    // Goal Heading RMP (Base)
    EuclideanGoalRmp goal_with_heading_rmp(
        gtsam_current_pose_in_fixed_, velocity_2d_, gtsam_current_goal_in_fixed_,
        controller_params_.euclidean_goal_gain_, controller_params_.euclidean_goal_offset_,
        controller_params_.euclidean_goal_steepness_, controller_params_.euclidean_goal_weight_);
    AccelerationVisualization acc_goal;
    acc_goal.id_ = "euclidean_goal";
    acc_goal.acc_ = flip_front_direction * goal_with_heading_rmp.acceleration().head<2>();
    acc_goal.angular_acc_ = flip_front_angle * goal_with_heading_rmp.acceleration()(2);
    acc_goal.metric_ = goal_with_heading_rmp.metric().block(0, 0, 2, 2);
    acc_goal.point_ = Eigen::Vector2d(0.0, 0.0);
    acc_goal.color_ = Eigen::Vector3f(1.0, 0.5, 0.0);
    acc_goal.weight_ = controller_params_.euclidean_goal_weight_;
    vis_accelerations_.push_back(acc_goal);

    // // Heading RMP
    double goal_distance = grid_map_.atPosition("geodesic", gtsam_current_pose_in_fixed_.translation());
    HeadingRmp heading_rmp(
        gtsam_current_pose_in_fixed_, velocity_2d_, Vector2(1.0, 0.0), goal_distance,
        controller_params_.heading_gain_, controller_params_.heading_offset_,
        controller_params_.heading_steepness_, controller_params_.heading_weight_);
    AccelerationVisualization acc_heading;
    acc_heading.id_ = "heading";
    acc_heading.acc_ = flip_front_direction * heading_rmp.acceleration().head<2>();
    acc_heading.angular_acc_ = flip_front_angle * heading_rmp.acceleration()(2);
    acc_heading.metric_ = heading_rmp.metric().block(0, 0, 2, 2);
    acc_heading.point_ = Eigen::Vector2d(0.0, 0.0);
    acc_heading.color_ = Eigen::Vector3f(0.0, 1.0, 0.0);
    acc_heading.weight_ = controller_params_.heading_weight_;
    vis_accelerations_.push_back(acc_heading);

    // Add RMPs
    if (params_.differential_drive_)
    {
        // Goal RMP
        // problem.addRmp(applyDifferentialModel(motion_model_, acc_diff), goal_rmp);

        // Regularization RMP
        Vector2 optimal_acc_diff = Vector2::Zero();
        optimal_acc_diff(0) = optimal_acc_(0);
        optimal_acc_diff(1) = optimal_acc_(2);
        problem.addRmp(acc_diff,
                       Regularization2Rmp(optimal_acc_diff, controller_params_.regularization_));

        // Damping RMP
        problem.addRmp(applyDifferentialModel(motion_model_, acc_diff), damping_rmp);

        // Goal Heading RMP
        problem.addRmp(applyDifferentialModel(motion_model_, acc_diff), goal_with_heading_rmp);

        // Heading RMP
        problem.addRmp(applyDifferentialModel(motion_model_, acc_diff), heading_rmp);
    }
    else
    {
        // Goal RMP
        // problem.addRmp(acc_se2, goal_rmp);

        // Regularization RMP
        problem.addRmp(acc_se2,
                       Regularization3Rmp(optimal_acc_, controller_params_.regularization_));

        // Damping RMP
        problem.addRmp(acc_se2, damping_rmp);

        // Goal Heading RMP
        problem.addRmp(acc_se2, goal_with_heading_rmp);

        // Heading RMP
        problem.addRmp(acc_se2, heading_rmp);
    }
    profiler_ptr_->endEvent("1.3.1.prepare_rmps");

    // ------------------------------------------------------------------------------------
    // Solve RMP general problem
    // ------------------------------------------------------------------------------------
    profiler_ptr_->startEvent("1.3.2.solve_rmps");
    Values solution;
    bool valid_solution = true;
    try
    {
        if (params_.differential_drive_)
        {
            solution = problem.solve(initial_diff_acc, true);
        }
        else
        {
            solution = problem.solve(initial_se2_acc, true);
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Error thrown during optimization of RMP problem:\n"
                         << std::string(e.what()));
        optimal_acc_ = Eigen::Vector3d::Zero();
        optimal_metric_ = Eigen::Matrix3d::Identity();
        valid_solution = false;
    }

    // Extract solution
    if (valid_solution)
    {
        if (params_.differential_drive_)
        {
            Vector2 optimal_acc_diff = solution.at<Vector2>(acc_diff_key);
            Matrix2 optimal_metric_diff = problem.metric();

            // Fill optimal acceleration
            optimal_acc_(0) = optimal_acc_diff(0);
            optimal_acc_(1) = 0.0;
            optimal_acc_(2) = optimal_acc_diff(1);

            // Fill optimal metric
            optimal_metric_ = Eigen::Matrix3d::Identity();
            optimal_metric_(0, 0) = optimal_metric_diff(0, 0);
            optimal_metric_(0, 2) = optimal_metric_diff(0, 1);
            optimal_metric_(2, 0) = optimal_metric_diff(1, 0);
            optimal_metric_(2, 2) = optimal_metric_diff(1, 1);
        }
        else
        {
            optimal_acc_ = solution.at<Vector3>(acc_se2_key);
            optimal_metric_ = problem.metric();
        }
    } else {
      ROS_WARN("Ivalid solution");
    }

    profiler_ptr_->endEvent("1.3.2.solve_rmps");

    // ------------------------------------------------------------------------------------
    // Integrate acceleration
    // ------------------------------------------------------------------------------------
    optimal_velocity_ = optimal_acc_ * controller_params_.integration_time_;

    ROS_INFO_STREAM_THROTTLE(1, "Optimal acceleration: \n" << optimal_acc_.transpose());
    ROS_INFO_STREAM_THROTTLE(1, "Optimal metric:   \n" << optimal_metric_);
    ROS_INFO_STREAM_THROTTLE(1, "Optimal velocity: \n" << optimal_velocity_.transpose());
}

//-------------------------------------------------------------------------------------------------
// Visualizations
//-------------------------------------------------------------------------------------------------
void RmpController::setupVisualizations(ros::NodeHandle& node_handle)
{
    control_points_pub_ = node_handle.advertise<visualization_msgs::MarkerArray>(
        "/locally_reactive_controller/rmp/control_points", 10);
    accelerations_pub_ = node_handle.advertise<visualization_msgs::MarkerArray>(
        "/locally_reactive_controller/rmp/accelerations", 10);
}

void RmpController::publishVisualizations(const std_msgs::Header& header)
{
    // Publish Control points (base)
    visualization_msgs::MarkerArray control_points_array =
        controlPointsToMarkerArray(control_points_, header);
    control_points_pub_.publish(control_points_array);

    // // Publish accelerations
    visualization_msgs::MarkerArray accs_array =
        accelerationsToMarkerArray(vis_accelerations_, header);
    accelerations_pub_.publish(accs_array);
}

visualization_msgs::MarkerArray RmpController::controlPointsToMarkerArray(
    const std::vector<ControlPointStruct>& control_points, const std_msgs::Header& header)
{
    int i = 0;
    visualization_msgs::MarkerArray cp_vis;
    for (auto cp : control_points)
    {
        // Control point marker
        visualization_msgs::Marker marker_cp;
        marker_cp.header = header;
        marker_cp.header.frame_id = params_.base_frame_;
        marker_cp.ns = "control_point";
        marker_cp.id = i;
        marker_cp.type = visualization_msgs::Marker::SPHERE;
        marker_cp.action = visualization_msgs::Marker::ADD;
        marker_cp.pose.position.x = cp.point_.x();
        marker_cp.pose.position.y = cp.point_.y();
        marker_cp.pose.position.z = 0.0;
        marker_cp.pose.orientation.x = 0.0;
        marker_cp.pose.orientation.y = 0.0;
        marker_cp.pose.orientation.z = 0.0;
        marker_cp.pose.orientation.w = 1.0;
        marker_cp.scale.x = 2 * cp.radius_;
        marker_cp.scale.y = 2 * cp.radius_;
        marker_cp.scale.z = 2 * cp.radius_;
        marker_cp.color.r = cp.color_(0);
        marker_cp.color.g = cp.color_(1);
        marker_cp.color.b = cp.color_(2);
        marker_cp.color.a = 0.2;
        // Add marker
        cp_vis.markers.push_back(marker_cp);

        // Update ids
        i++;
    }

    return cp_vis;
}

visualization_msgs::MarkerArray RmpController::accelerationsToMarkerArray(
    const std::vector<AccelerationVisualization>& accelerations, const std_msgs::Header& header)
{
    int i = 0;
    visualization_msgs::MarkerArray acc_vis;

    for (auto acc : accelerations)
    {
        // Accelerations
        visualization_msgs::Marker marker_acc;
        {
            marker_acc.header = header;
            marker_acc.header.frame_id = params_.base_frame_;
            marker_acc.ns = "acc_" + acc.id_;
            marker_acc.id = i;
            marker_acc.type = visualization_msgs::Marker::ARROW;
            marker_acc.action = visualization_msgs::Marker::ADD;
            marker_acc.pose.position.x = acc.point_.x();
            marker_acc.pose.position.y = acc.point_.y();
            marker_acc.pose.position.z = 0.0;
            marker_acc.pose.orientation.x = 0.0;
            marker_acc.pose.orientation.y = 0.0;
            marker_acc.pose.orientation.z = 0.0;
            marker_acc.pose.orientation.w = 1.0;
            marker_acc.scale.x = 0.02;
            marker_acc.scale.y = 0.05;
            marker_acc.scale.z = 0.05;
            marker_acc.color.r = acc.color_(0);
            marker_acc.color.g = acc.color_(1);
            marker_acc.color.b = acc.color_(2);
            marker_acc.color.a = 1.0;
            geometry_msgs::Point p1;
            p1.x = 0.0;
            p1.y = 0.0;
            p1.z = 0.0;
            geometry_msgs::Point p2;
            p2.x = acc.acc_.x() * 0.05 * acc.weight_;
            p2.y = acc.acc_.y() * 0.05 * acc.weight_;
            p2.z = 0.0;
            marker_acc.points.push_back(p1);
            marker_acc.points.push_back(p2);
        }
        // Add marker
        acc_vis.markers.push_back(marker_acc);

        // Angular accelerations
        visualization_msgs::Marker marker_ang_acc;
        {
            marker_ang_acc.header = header;
            marker_ang_acc.header.frame_id = params_.base_frame_;
            marker_ang_acc.ns = "angular_acc_" + acc.id_;
            marker_ang_acc.id = i;
            marker_ang_acc.type = visualization_msgs::Marker::ARROW;
            marker_ang_acc.action = visualization_msgs::Marker::ADD;
            marker_ang_acc.pose.position.x = acc.point_.x();
            marker_ang_acc.pose.position.y = acc.point_.y();
            marker_ang_acc.pose.position.z = 0.0;
            marker_ang_acc.pose.orientation.x = 0.0;
            marker_ang_acc.pose.orientation.y = 0.0;
            marker_ang_acc.pose.orientation.z = 0.0;
            marker_ang_acc.pose.orientation.w = 1.0;
            marker_ang_acc.scale.x = 0.04;
            marker_ang_acc.scale.y = 0.07;
            marker_ang_acc.scale.z = 0.07;
            marker_ang_acc.color.r = acc.color_(0);
            marker_ang_acc.color.g = acc.color_(1);
            marker_ang_acc.color.b = acc.color_(2);
            marker_ang_acc.color.a = 1.0;
            geometry_msgs::Point p1;
            p1.x = 0.0;
            p1.y = 0.0;
            p1.z = 0.0;
            geometry_msgs::Point p2;
            p2.x = 0.0;
            p2.y = acc.angular_acc_ * 0.05 * acc.weight_;
            p2.z = 0.0;
            marker_ang_acc.points.push_back(p1);
            marker_ang_acc.points.push_back(p2);
        }
        // Add marker
        acc_vis.markers.push_back(marker_ang_acc);

        // Metric
        // To plot the metric we use supereight_atlas' approach
        // https://github.com/ori-drs/supereight_atlas/blob/master/se_atlas_ros/src/ros/ros_pub.cpp#L1157
        // We first need to compute the eigenvectors and eigenvalues
        Eigen::Vector2d vis_eigenvalues(Eigen::Vector2d::Zero());
        Eigen::Matrix2d vis_eigenvectors(Eigen::Matrix2d::Zero());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> vis_solver(acc.metric_);

        if (vis_solver.info() == Eigen::Success)
        {
            vis_eigenvalues = vis_solver.eigenvalues();
            vis_eigenvectors = vis_solver.eigenvectors();
            // ROS_INFO_STREAM_THROTTLE(1, "eigen vectors: \n" << vis_eigenvectors);
            // ROS_INFO_STREAM_THROTTLE(1, "eigen values:  \n" << vis_eigenvalues);
        }
        else
        {
            ROS_WARN_STREAM("Couldn't compute eigenvectors for metric of acc [" << acc.id_ << "]");
            continue;
        }
        // Create rotation matrix from eigenvectors
        Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
        rot.block(0, 0, 2, 2) = vis_eigenvectors;

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translate(Eigen::Vector3d(acc.point_.x(), acc.point_.y(), 0.0));
        pose.rotate(rot);

        geometry_msgs::Pose metric_pose;
        tf::poseEigenToMsg(pose, metric_pose);

        // Create marker
        visualization_msgs::Marker marker_metric;
        {
            marker_metric.header = header;
            marker_metric.header.frame_id = params_.base_frame_;
            marker_metric.ns = "metrics_" + acc.id_;
            marker_metric.id = i;
            marker_metric.type = visualization_msgs::Marker::SPHERE;
            marker_metric.action = visualization_msgs::Marker::ADD;
            marker_metric.pose = metric_pose;
            marker_metric.scale.x = sqrt(vis_eigenvalues[0]) * 1.0;
            marker_metric.scale.y = sqrt(vis_eigenvalues[1]) * 1.0;
            marker_metric.scale.z = 0.1;
            marker_metric.color.r = acc.color_(0);
            marker_metric.color.g = acc.color_(1);
            marker_metric.color.b = acc.color_(2);
            marker_metric.color.a = 0.2;
        }
        // Add marker
        acc_vis.markers.push_back(marker_metric);

        // Update ids
        i++;
    }

    return acc_vis;
}