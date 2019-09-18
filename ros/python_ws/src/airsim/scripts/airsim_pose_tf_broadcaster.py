#!/usr/bin/env python

import setup_path 
import airsim
import rospy
import roslib
import time
import tf2_ros
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
 

def handle_airsim_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "airSimPoseFrame"
    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z
    t.transform.rotation.x = msg.pose.orientation.x
    t.transform.rotation.y = msg.pose.orientation.y
    t.transform.rotation.z = msg.pose.orientation.z
    t.transform.rotation.w = msg.pose.orientation.w
    br.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('airsim_pose_tf_broadcaster')
    rospy.Subscriber('airsimPose',PoseStamped, handle_airsim_pose)
    rospy.spin()

