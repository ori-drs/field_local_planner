#!/usr/bin/env python

import rospy
import rospkg
import tf.transformations as tr
import math
import time
import numpy as np
import os
import pathlib

from locally_reactive_controller_evaluation import robot_geometry
from locally_reactive_controller_evaluation import test_trajectory
from locally_reactive_controller_msgs.msg import Status
from high_level_actions import *

# Services
from any_gazebo_msgs.srv import SetRobotPose, SetRobotPoseRequest
from std_srvs.srv import Empty, EmptyRequest
from motion_transitioner_msgs.srv import SetCurrentMotionState, SetCurrentMotionStateRequest
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest
from gazebo_msgs.srv import GetPhysicsProperties, GetPhysicsPropertiesRequest, GetPhysicsPropertiesResponse

# Messages
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import Int16
from gazebo_msgs.msg import ContactsState
from visualization_msgs.msg import Marker

class ExperimentHandler:
    def __init__(self):
        # Parameters
        self.timeout =  120 # 1200 for the small environment # seconds

        self.fixed_frame = "odom"
        self.waypoint_threshold = 1.0 # meters
        self.waypoint_angle_threshold = 120.0*math.pi/180.0 # radians

        self.goal_threshold = 0.21 # meters
        self.goal_angle_threshold = 0.1 # radians

        self.simulation_speed = 20 # percent

        self.collision_timeout = 0.3 # secs

        self.progress_distance_thr = 0.4
        self.progress_timeout = 5
        
        
        
        # Internal
        self.test_poses = test_trajectory.test_poses
        self.initial_pose = self.test_poses[0]
        self.waypoint_pose = self.test_poses[1]
        self.waypoint_idx = 1
        self.last_waypoint = len(self.test_poses) - 1
        self.trial = 1
        self.is_running = True
        self.t_trial_initial = None
        self.robot_poses = []
        self.robot_twists = []
        self.robot_collisions = []
        self.robot_pose = None
        self.last_robot_pose_stuck = None
        self.robot_twist = None
        self.robot_collision = None
        self.trial_time = None
        self.time_since_last_progress_check = None

        self.time_since_last_collision = None
        self.colliding = False
        self.collision_counter = 0

        # Configure paths
        rospack = rospkg.RosPack()
        self.results_path = rospack.get_path("locally_reactive_controller_evaluation") + "/results/"
        
        # Setup
        self.read_parameters()
        self.setup_ros()
        self.reset_trial()
        self.publish_testing_path()
    
    def read_parameters(self):
        # Read parameters
        self.trial             = rospy.get_param("~initial_trial", 1)
        self.max_trials        = rospy.get_param("~max_trials", 10)
        self.goal_topic        = rospy.get_param("~goal_topic", "/goal")
        self.robot_pose_topic  = rospy.get_param("~robot_pose_topic", "/state_estimator/pose_in_odom")
        self.robot_twist_topic = rospy.get_param("~robot_twist_topic", "/state_estimator/twist")
        self.collisions_topic  = rospy.get_param("~world_collisions_topic", "/gazebo/world_collisions")

        self.collisions_obstacle1_topic  = rospy.get_param("~obstacle1_collisions_topic", "/gazebo/obstacle1_link_collisions")
        self.collisions_obstacle2_topic  = rospy.get_param("~obstacle2_collisions_topic", "/gazebo/obstacle2_link_collisions")
        self.collisions_obstacle3_topic  = rospy.get_param("~obstacle3_collisions_topic", "/gazebo/obstacle3_link_collisions")
        self.collisions_obstacle4_topic  = rospy.get_param("~obstacle4_collisions_topic", "/gazebo/obstacle4_link_collisions")

        self.testing_controller_name = rospy.get_param("/locally_reactive_controller/name", None)
    
    def setup_ros(self):
        # Setup subscribers
        self.pose_sub = rospy.Subscriber(self.robot_pose_topic, PoseWithCovarianceStamped, self.robot_pose_callback, queue_size=1)
        self.twist_sub = rospy.Subscriber(self.robot_twist_topic, TwistWithCovarianceStamped, self.robot_twist_callback, queue_size=1)
        self.collisions_sub = rospy.Subscriber(self.collisions_topic, ContactsState, self.robot_collision_callback, queue_size=1)

        self.collisions_obstacle1_sub = rospy.Subscriber(self.collisions_obstacle1_topic, ContactsState, self.robot_collision_callback, queue_size=1)
        self.collisions_obstacle2_sub = rospy.Subscriber(self.collisions_obstacle2_topic, ContactsState, self.robot_collision_callback, queue_size=1)
        self.collisions_obstacle3_sub = rospy.Subscriber(self.collisions_obstacle3_topic, ContactsState, self.robot_collision_callback, queue_size=1)
        self.collisions_obstacle4_sub = rospy.Subscriber(self.collisions_obstacle4_topic, ContactsState, self.robot_collision_callback, queue_size=1)

        # Setup publishers
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
        self.stop_walking_pub = rospy.Publisher("/locally_reactive_controller/stop_walking_cmd", Int16, queue_size=10)
        self.testing_path_pub = rospy.Publisher("/simulation_experiment/testing_path", Path, queue_size=10)
        self.collisions_counter_pub = rospy.Publisher("/simulation_experiment/collisions_counter", Marker, queue_size=10)

        self.current_path_pubs = []
        self.current_path_pubs.append(None)
        for i in range(1, self.max_trials + 1):
            self.current_path_pubs.append(rospy.Publisher("/paths_%s%i" % (self.testing_controller_name, i), Path, queue_size=10))

        # Setup services
        # Simulation speed
        gazebo_physics_response = GetPhysicsPropertiesResponse()
        try:
            gazebo_physics_request = GetPhysicsPropertiesRequest()
            gazebo_physics_service = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)
            gazebo_physics_response = gazebo_physics_service(gazebo_physics_request)
        except rospy.ServiceException as e:
            rospy.logwarn("Could not set the simulation speed")
        try:
            gazebo_physics_set_request = SetPhysicsPropertiesRequest()
            gazebo_physics_set_request.gravity = gazebo_physics_response.gravity
            gazebo_physics_set_request.time_step = gazebo_physics_response.time_step
            gazebo_physics_set_request.ode_config = gazebo_physics_response.ode_config
            gazebo_physics_set_request.max_update_rate = 10 * self.simulation_speed # 400 is 40%
            gazebo_physics_set_service = rospy.ServiceProxy("/gazebo/set_physics_properties", SetPhysicsProperties)
            resp1 = gazebo_physics_set_service(gazebo_physics_set_request)
        except rospy.ServiceException as e:
            rospy.logwarn("Could not set the simulation speed")
    
    # Callbacks
    def robot_pose_callback(self, msg):
        t = msg.header.stamp

        if self.t_trial_initial is None:
            return
        # rospy.logwarn("New pose")
        if self.last_robot_pose_stuck == None:
            self.last_robot_pose_stuck = msg.pose.pose

        # Update robot pose
        self.robot_pose = msg.pose.pose
        # Save current pose
        
        self.robot_poses.append({'stamp': (t - self.t_trial_initial).to_sec(), 'pose': msg.pose.pose})

    def robot_twist_callback(self, msg):
        t = msg.header.stamp

        if self.t_trial_initial is None:
            return
        # rospy.logwarn("New twist")
        self.robot_twist = msg.twist.twist
        # Save current velocity        
        self.robot_twists.append({'stamp': (t - self.t_trial_initial).to_sec(), 'twist': msg.twist.twist})

    def robot_collision_callback(self, msg):
        t = msg.header.stamp
        print_warn = True

        if self.t_trial_initial is None:
            return
        a = ContactsState()
        for s in msg.states:
            # We ignore foot collisions
            if "FOOT" in s.collision1_name:
                continue

            if not self.colliding:
                self.colliding = True
                rospy.logwarn("collision with %s" + s.collision1_name)
                # Save current collision
                self.time_since_last_collision = t - self.t_trial_initial
                # Store collision event
                self.robot_collisions.append({'stamp': (self.time_since_last_collision).to_sec(), 'collision': 1})
                self.collision_counter+=1
                return

    def run(self):
        r = rospy.Rate(10)
        t_start = time.time()
        while not rospy.is_shutdown():
            if self.is_running:
                self.is_running = self.run_experiment()
            
            # If the experiment is paused or not running
            else:
                t_end = time.time()
                rospy.logwarn("Trials Finished in %.3f" % (t_end - t_start))
                break
            r.sleep()
    
    def run_experiment(self):
        keep_running = False

        # Publish reference path
        self.publish_testing_path()
        self.publish_current_path()
        # Publish collisions
        self.check_and_publish_collisions()

        # Check trials
        # if self.trial == 0:
        #     self.reset_trial()
        # Check if we passed all the trials
        if self.trial > self.max_trials:
            return False
   
        # Run trial
        trial_done = self.run_trial()

        # Check timeout
        if trial_done:
            # Save data and start new trial
            self.export_data()
            self.reset_trial()
            self.next_trial()
        
                
        return True
    
    def export_data(self):
        controller_path = self.results_path + self.testing_controller_name + "/"

        # Check if folder for controller exists
        pathlib.Path(controller_path).mkdir(parents=True, exist_ok=True)

        # Open file to save poses in TUM format
        poses_file = controller_path + "poses_" + str(self.trial) + ".csv"
        with open(poses_file, 'w') as f:
            for pose in self.robot_poses:
                t = pose["stamp"]
                T = pose["pose"]
                f.writelines("%.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f\n" % 
                    (t, T.position.x, T.position.y, T.position.z, 
                        T.orientation.x, T.orientation.y, T.orientation.z, T.orientation.w))
        

        # Open file to save twists
        twists_file = controller_path + "twists_" + str(self.trial) + ".csv"
        with open(twists_file, 'w') as f:
            for twist in self.robot_twists:
                t = twist["stamp"]
                v = twist["twist"]
                f.writelines("%.8f %.8f %.8f %.8f %.8f %.8f %.8f\n" % 
                    (t, v.linear.x, v.linear.y, v.linear.z, 
                        v.angular.x, v.angular.y, v.angular.z))

        # Open file to save contacts
        collisions_file = controller_path + "collisions_" + str(self.trial) + ".csv"
        with open(collisions_file, 'w') as f:
            for coll in self.robot_collisions:
                t = coll["stamp"]
                c = coll["collision"]
                f.writelines("%.8f %.8f\n" % (t, c))
        rospy.loginfo("Data for trial %i exported!" % self.trial)

    def reset_experiment(self):
        self.reset_robot_pose()
    
    def reset_robot_pose(self):
        self.resetted = True

        # Send stop command to position controller
        pub = rospy.Publisher(
            '/locally_reactive_controller/stop_walking_cmd', Int16, queue_size=10)
        msg = Int16()
        pub.publish(msg)
        time.sleep(10)

        # Change state so stand
        rospy.logwarn("STAND")
        switch_controller("dynamic_gaits")
        switch_dynamic_gaits_gait("stand")
        time.sleep(2)

        # Call emergency stop
        rospy.logwarn("E-STOP")
        try:
            estop_request = EmptyRequest()
            estop_service = rospy.ServiceProxy("/motion_control_manager/emergency_stop", Empty)
            resp1 = estop_service(estop_request)
        except rospy.ServiceException as e:
            rospy.logerr("FAILED to enable emergency stop")

        time.sleep(2)

        # Reset robot pose
        rospy.logwarn("RESET robot pose")
        try:
            reset_pose_request = SetRobotPoseRequest()
            reset_pose_request.pose = self.initial_pose
            reset_pose_request.pose.position.z += 0.01
            reset_pose_service = rospy.ServiceProxy("/gazebo/set_robot_pose", SetRobotPose)
            resp1 = reset_pose_service(reset_pose_request)
        except rospy.ServiceException as e:
            rospy.logerr("FAILED to reset robot pose")
        
        time.sleep(2)

        # Reset state estimator
        # Not applicabble in simulation

        # Clear elevation map
        rospy.logwarn("CLEAR elevation map")
        try:
            estop_request = EmptyRequest()
            estop_service = rospy.ServiceProxy("/elevation_mapping/clear_map", Empty)
            resp1 = estop_service(estop_request)
        except rospy.ServiceException as e:
            rospy.logerr("FAILED to clear elevation map")
        time.sleep(1)

        # Reset position controller
        # Apparently not necessary

        # Clear emergency stop
        rospy.logwarn("CLEAR E-STOP")
        try:
            estop_request = EmptyRequest()
            estop_service = rospy.ServiceProxy("/motion_control_manager/clear_emergency_stop", Empty)
            resp1 = estop_service(estop_request)
        except rospy.ServiceException as e:
            rospy.logerr("FAILED to clear emergency stop")

        time.sleep(2)

        # Change controller to walk
        rospy.logwarn("DYNAMIC GAITS - WALK")
        switch_controller("dynamic_gaits")
        switch_dynamic_gaits_gait("walk")

    def set_walking_state(self):
        # Clear estop

        # Set walking mode
        try:
            motion_state_request = SetCurrentMotionStateRequest()
            motion_state_request.motion_state_name.data = "stand"
            motion_state_service = rospy.ServiceProxy("/motion_control_manager/set_current_motion_state", SetCurrentMotionState)
            resp1 = motion_state_service(motion_state_request)
            return True
        except rospy.ServiceException as e:
            return False
    
    def run_trial(self):
        # Get current ROS time
        t = rospy.get_rostime()
        self.t_trial_initial = t if self.t_trial_initial is None else self.t_trial_initial
        # Compute dt
        self.trial_time = (t - self.t_trial_initial)

        # Publish current goal/waypoint
        goal = PoseStamped()
        goal.header.frame_id = self.fixed_frame
        goal.header.stamp = rospy.Time.now()
        goal.pose = self.waypoint_pose
        self.goal_pub.publish(goal)
        rospy.loginfo_throttle(10, "[Setup %s | Trial %i/%i | Waypoint %i/%i | %.2f/%.2f] Waypoint (%f, %f)" % 
                                (self.testing_controller_name, self.trial, self.max_trials,
                                self.waypoint_idx, self.last_waypoint,
                                self.trial_time.to_sec(), self.timeout,
                                self.waypoint_pose.position.x, self.waypoint_pose.position.y))

        # Check timeout
        if self.trial_time.to_sec() > self.timeout:
            return True
        if self.robot_pose is None:
            return False
        
        # check progress
        if self.time_since_last_progress_check is None:
            self.time_since_last_progress_check = self.trial_time

        if (self.trial_time - self.time_since_last_progress_check).to_sec() > self.progress_timeout:
            # Check if the robot is in the same position
            error_progress = math.sqrt((self.robot_pose.position.x - self.last_robot_pose_stuck.position.x)**2 + 
                                       (self.robot_pose.position.y - self.last_robot_pose_stuck.position.y)**2)
            
            if (error_progress <= self.progress_distance_thr):
                rospy.logwarn("Resetting trial. Robot stuck")
                return True
            else:
                self.last_robot_pose_stuck = self.robot_pose
                self.time_since_last_progress_check = self.trial_time
                rospy.logwarn("Restarting progress check")
        
        # Check terminal conditions
        # Compute orientation w.r.t waypoint
        q_robot = np.array([self.robot_pose.orientation.x,
                            self.robot_pose.orientation.y,
                            self.robot_pose.orientation.z,
                            self.robot_pose.orientation.w])
        q_waypoint  = np.array([self.waypoint_pose.orientation.x,
                                self.waypoint_pose.orientation.y,
                                self.waypoint_pose.orientation.z,
                                self.waypoint_pose.orientation.w])
        # Change euler angles
        eul = (tr.euler_from_quaternion(tr.quaternion_multiply(tr.quaternion_inverse(q_waypoint), q_robot)))
        error_angle = math.fabs(eul[2])
        
        error_distance = math.sqrt((self.robot_pose.position.x - self.waypoint_pose.position.x)**2 + 
                                   (self.robot_pose.position.y - self.waypoint_pose.position.y)**2)
        
        goal_pose = self.test_poses[self.last_waypoint]
        goal_error_distance = math.sqrt((self.robot_pose.position.x - goal_pose.position.x)**2 + 
                                        (self.robot_pose.position.y - goal_pose.position.y)**2)

        # Check if arrived at final goal
        if self.waypoint_idx == self.last_waypoint:
            if ((goal_error_distance < self.goal_threshold) and (goal_error_distance < self.goal_angle_threshold)):
                # Finish
                rospy.loginfo("[Setup %s | Trial %i/%i | Waypoint %i/%i | %.2f/%.2f] Robot arrived at goal position" % 
                                (self.testing_controller_name, self.trial, self.max_trials,
                                self.waypoint_idx, self.last_waypoint,
                                self.trial_time.to_sec(), self.timeout))
                return True

        # Check if arrived at waypoint
        else:
            if ((error_distance < self.waypoint_threshold) and (error_angle < self.waypoint_angle_threshold)):
                rospy.loginfo("[Setup %s | Trial %i/%i | Waypoint %i/%i | %.2f/%.2f] Robot arrived at waypoint position (%f, %f)" % 
                                (self.testing_controller_name, self.trial, self.max_trials,
                                self.waypoint_idx, self.last_waypoint,
                                self.trial_time.to_sec(), self.timeout,
                                self.waypoint_pose.position.x, self.waypoint_pose.position.y))

                # Update waypoint index
                found_waypoint = False
                for i in range(self.waypoint_idx, self.last_waypoint):
                    candidate_waypoint_pose = self.test_poses[i]
                    error_distance = math.sqrt((self.robot_pose.position.x - candidate_waypoint_pose.position.x)**2 + 
                                               (self.robot_pose.position.y - candidate_waypoint_pose.position.y)**2)
                    if ((error_distance < self.waypoint_threshold) and (error_angle < self.waypoint_angle_threshold)):
                        continue
                    else:
                        self.waypoint_idx = i
                        found_waypoint = True
                        break
                
                if not found_waypoint:
                    self.waypoint_idx = self.last_waypoint

                # self.waypoint_idx +=1
                                
                # Update waypoint pose
                self.waypoint_pose = self.test_poses[self.waypoint_idx]

                # Show next waypoint
                rospy.loginfo("[Setup %s | Trial %i | Waypoint %i/%i | %.2f/%.2f] Next waypoint (%f, %f)" % 
                                (self.testing_controller_name, self.trial,
                                self.waypoint_idx, self.last_waypoint,
                                self.trial_time.to_sec(), self.timeout,
                                self.waypoint_pose.position.x, self.waypoint_pose.position.y))
                

        return False # has not finished

    def reset_trial(self):
        rospy.loginfo("[Setup %s] Resetting" % (self.testing_controller_name))
        self.reset_robot_pose()
        self.set_walking_state()

        self.waypoint_idx = 1
        self.waypoint_pose = self.test_poses[self.waypoint_idx]
        
        self.t_trial_initial = None
        self.robot_poses = []
        self.robot_twists = []
        self.robot_collisions = []
        self.robot_pose = None
        self.last_robot_pose_stuck = None
        self.robot_twist = None
        self.robot_collision = None
        self.trial_time = None
        self.time_since_last_progress_check = None

        self.time_since_last_collision = None
        self.colliding = False
        self.collision_counter = 0

    def next_trial(self):
        self.trial = self.trial + 1
        if self.trial < self.max_trials:
            rospy.loginfo("[Setup %s | Trial %i] Starting new trial" % (self.testing_controller_name, self.trial))
    
    def publish_testing_path(self):
        path_msg = Path()
        for T in test_trajectory.test_poses:
            pose_stamped = PoseStamped()
            pose_stamped.pose = T
            pose_stamped.header.frame_id = self.fixed_frame
            pose_stamped.header.stamp = rospy.Time.now()
            path_msg.poses.append(pose_stamped)
        path_msg.header.frame_id = self.fixed_frame
        path_msg.header.stamp = rospy.Time.now()
        self.testing_path_pub.publish(path_msg)
    
    def publish_current_path(self):
        path_msg = Path()
        for T in self.robot_poses:
            pose_stamped = PoseStamped()
            pose_stamped.pose = T["pose"]
            pose_stamped.header.frame_id = self.fixed_frame
            pose_stamped.header.stamp = rospy.Time.now()
            path_msg.poses.append(pose_stamped)
        path_msg.header.frame_id = self.fixed_frame
        path_msg.header.stamp = rospy.Time.now()
        if self.trial <= self.max_trials:
            self.current_path_pubs[self.trial].publish(path_msg)
    
    def check_and_publish_collisions(self):

        # Check if the collision stopped
        if (self.time_since_last_collision is not None):
            if (self.colliding and (self.trial_time - self.time_since_last_collision).to_sec() > self.collision_timeout):
                self.colliding = False

        marker_msg = Marker()
        marker_msg.header.frame_id = self.fixed_frame
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.type = marker_msg.TEXT_VIEW_FACING
        marker_msg.action = marker_msg.ADD
        marker_msg.ns = "Collision Info"
        marker_msg.text = "Collisions: %i" % self.collision_counter
        marker_msg.scale.x = 0.2
        marker_msg.scale.y = 0.2
        marker_msg.scale.z = 0.3
        if self.colliding:
            marker_msg.color.r = 1.0
            marker_msg.color.g = 0.0
            marker_msg.color.b = 0.0
            marker_msg.color.a = 1.0
        else:
            marker_msg.color.r = 0.0
            marker_msg.color.g = 0.0
            marker_msg.color.b = 0.0
            marker_msg.color.a = 1.0
        marker_msg.pose.orientation.w = 1.0
        marker_msg.pose.position.x = -2.7
        marker_msg.pose.position.y = -4.1
        marker_msg.pose.position.z = 2.2
        
        self.collisions_counter_pub.publish(marker_msg)
    
if __name__ == "__main__":
    rospy.init_node("simulation_experiment")

    handler = ExperimentHandler()
    handler.run()
    rospy.spin()