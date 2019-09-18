#!/usr/bin/env python

import setup_path 
import airsim
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import numpy as np


def imu_airpub(frameID, pubNode, sensorName, vehicleName):
    pub = rospy.Publisher(pubNode, Imu, queue_size=1)
    rospy.init_node('airsim_imu_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # connect to the AirSim simulator 
    client = airsim.CarClient()
    client.confirmConnection()

    while not rospy.is_shutdown():
        
         # get data of IMU sensor
        imuData = client.getImuData(sensorName, vehicleName)

        # populate Imu ros message
        imu_msg = Imu()
        imu_msg.header.frame_id = frameID
        imu_msg.orientation.x = imuData.orientation.inverse().x_val
        imu_msg.orientation.y = imuData.orientation.inverse().y_val
        imu_msg.orientation.z = imuData.orientation.inverse().z_val
        imu_msg.orientation.w = imuData.orientation.inverse().w_val

        imu_msg.angular_velocity.x = np.deg2rad(imuData.angular_velocity.x_val)
        imu_msg.angular_velocity.y = -np.deg2rad(imuData.angular_velocity.y_val)
        imu_msg.angular_velocity.z = -np.deg2rad(imuData.angular_velocity.z_val)

        imu_msg.linear_acceleration.x = imuData.linear_acceleration.x_val
        imu_msg.linear_acceleration.y = -imuData.linear_acceleration.y_val
        imu_msg.linear_acceleration.z = -imuData.linear_acceleration.z_val
        
        # log Imu message
        rospy.loginfo(imu_msg)
        #publish Imu message
        pub.publish(imu_msg)
        # sleeps until next cycle 
        rate.sleep()



if __name__ == '__main__':
    try:
        frameID = rospy.get_param('frame_id', 'airSimPoseFrame')
        pubNode =  rospy.get_param('pub_node', 'airsimIMU')
        sensorName =  rospy.get_param('sensor_name', 'imu')
        vehicleName = rospy.get_param('vehicle_name', 'vehicle')
        imu_airpub(frameID, pubNode, sensorName, vehicleName)
    except rospy.ROSInterruptException:
        pass
