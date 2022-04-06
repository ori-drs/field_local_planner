from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
import tf.transformations as tr
import numpy as np

def se3_matrix_to_ros_pose(T: np.array):
    # Convert SE(3) matrix to quaternion + translation
    q = tr.quaternion_from_matrix(T[0:3,0:3])

    # Fill message
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = "odom"
    pose.pose.position.x = T[0,3]
    pose.pose.position.y = T[1,3]
    pose.pose.position.z = T[2,3]
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose

def ros_pose_with_covariance_stamped_to_se3(pose: PoseWithCovarianceStamped):
    q = np.array([pose.pose.pose.orientation.x,
                  pose.pose.pose.orientation.y,
                  pose.pose.pose.orientation.z,
                  pose.pose.pose.orientation.w])
    p = np.array([pose.pose.pose.position.x,
                  pose.pose.pose.position.y,
                  pose.pose.pose.position.z])
    
    # Create SE(3) pose
    T = tr.quaternion_matrix(q)
    T[0:3,3] = p

    return T

def ros_pose_to_se3(pose: Pose):
    q = np.array([pose.orientation.x,
                  pose.orientation.y,
                  pose.orientation.z,
                  pose.orientation.w])
    p = np.array([pose.position.x,
                  pose.position.y,
                  pose.position.z])
    
    # Create SE(3) pose
    T = tr.quaternion_matrix(q)
    T[0:3,3] = p

    return T

def ros_twist_to_vector(twist: TwistWithCovarianceStamped):
    v = np.zeros((6,1))
    v[0] = twist.twist.twist.angular.x
    v[1] = twist.twist.twist.angular.y
    v[2] = twist.twist.twist.angular.z
    v[3] = twist.twist.twist.linear.x
    v[4] = twist.twist.twist.linear.y
    v[5] = twist.twist.twist.linear.z
    return v