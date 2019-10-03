#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import geometry_msgs.msg
 

def handle_airsim_pose(msg, args):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = args[0]
    t.child_frame_id = args[1]
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
    frameID = rospy.get_param('~frame_id', 'world')
    childFrameID = rospy.get_param('~child_frame_id', 'base_link')
    poseNode = rospy.get_param('~pose_node', 'airsim/car_pose')
    rospy.Subscriber(poseNode, geometry_msgs.msg.PoseStamped, handle_airsim_pose, (frameID, childFrameID))
    rospy.spin()

