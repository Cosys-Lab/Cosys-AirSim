#!/usr/bin/env python

import setup_path 
import airsim
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


def car_pose_airpub(frameID, pubNode, vehicleName):
    pub = rospy.Publisher(pubNode, PoseStamped, queue_size=1)
    rate = rospy.Rate(10) # 10hz

    # connect to the AirSim simulator 
    client = airsim.CarClient()
    client.confirmConnection()

    while not rospy.is_shutdown():

        # get state of the car
        car_state = client.getCarState(vehicleName)
        pos = car_state.kinematics_estimated.position
        orientation = car_state.kinematics_estimated.orientation.inverse()

        # populate PoseStamped ros message
        simPose = PoseStamped()
        simPose.pose.position.x = pos.x_val
        simPose.pose.position.y = -pos.y_val
        simPose.pose.position.z = -pos.z_val
        simPose.pose.orientation.w = orientation.w_val
        simPose.pose.orientation.x = orientation.x_val
        simPose.pose.orientation.y = orientation.y_val
        simPose.pose.orientation.z = orientation.z_val
        simPose.header.stamp = rospy.Time.now()
        simPose.header.seq = 1
        simPose.header.frame_id = frameID
        
        #publish PoseStamped message
        pub.publish(simPose)
        # sleeps until next cycle 
        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('airsim_car_pose', anonymous=True)
        frameID =rospy.get_param('~frame_id', 'world')
        pubNode = rospy.get_param('~pub_node', 'airsim/car_pose')
        vehicleName = rospy.get_param('~vehicle_name', 'airsimvehicle')
        car_pose_airpub(frameID, pubNode, vehicleName)
    except rospy.ROSInterruptException:
        pass
