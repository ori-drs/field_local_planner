#!/usr/bin/env python

import rospy
import rospkg
from field_local_planners_msgs.msg import Status
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import tf.transformations as tr
import math

# Global variables
rospack = rospkg.RosPack()
path_pkg = rospack.get_path('field_local_planners')

carrot_file = path_pkg + '/config/meshes/carrot/carrot.dae'
carrot_marker_pub = rospy.Publisher('/field_local_planners/real_carrot', Marker, queue_size=10)

carrot_rotation_frequency = 1.0 # Hz
goal_reached = False
controller_status = Status.STOPPED

def controller_status_callback(msg):
    global goal_reached
    global controller_status

    goal_reached = msg.goal_reached
    controller_status = msg.status

def goal_callback(msg):
    global goal_reached
    global controller_status

    t = msg.header.stamp.to_sec()
    yaw = math.fmod(carrot_rotation_frequency * t, 2* math.pi)
    q = tr.quaternion_about_axis(yaw, (0, 0, 1))

    scale = 0.3

    marker = Marker()
    marker.header = msg.header
    marker.id = 0
    marker.ns = 'carrot'
    if goal_reached:
        marker.action = Marker.DELETE
    else:
        marker.action = Marker.ADD
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.a = 1.0
    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]
    marker.pose.position = msg.pose.position
    if(marker.pose.position.z <= 1e-3):
        marker.pose.position.z = marker.pose.position.z + 0.5
    marker.mesh_use_embedded_materials = True
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_resource = 'file://' + carrot_file

    # Publish
    carrot_marker_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('field_local_planners_carrot_publisher')

    # Read parameter with mesh
    # world_option = rospy.get_param('~world', 'arena_simple')
   
    # Objects
    goal_topic = rospy.get_param('~goal_topic', '/field_local_planners/current_goal')
    rospy.Subscriber(goal_topic, PoseStamped, goal_callback, queue_size=10)

    controller_status_topic = rospy.get_param('~controller_status_topic', '/field_local_planners/controller_status')
    rospy.Subscriber(controller_status_topic, Status, controller_status_callback, queue_size=10)

    rospy.spin()