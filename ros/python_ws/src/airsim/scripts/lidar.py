#!/usr/bin/env python

import setup_path 
import airsimpy
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from airsim.msg import StringArray
import numpy as np
import time

def lidar_airpub(frameID, pubNode, pubNodeGT, sensorName, vehicleName):
    pub = rospy.Publisher(pubNode, PointCloud2, queue_size=1)
    pubGT = rospy.Publisher(pubNodeGT, StringArray, queue_size=1)

    rate = rospy.Rate(10) # 10hz

    # connect to the AirSim simulator 
    client = airsimpy.CarClient()
    client.confirmConnection(rospy.get_name())

    lastTimestamp = None
    fullScan = True

    while not rospy.is_shutdown():

        # initiate point cloud
        pcloud = PointCloud2()
        groundtruth = StringArray()

        # get lidar data
        lidarData = client.getLidarData(sensorName, vehicleName)

        if lidarData.time_stamp != lastTimestamp:
            #Check if there are any points in the data
            if (len(lidarData.point_cloud) < 4):
                lastTimestamp = lidarData.time_stamp
            else:
                lastTimestamp = lidarData.time_stamp
                labels = np.array(lidarData.groundtruth, dtype=np.dtype('U'))
                points = np.array(lidarData.point_cloud, dtype=np.dtype('f4'))
                points = np.reshape(points, (int(points.shape[0] / 3), 3))
                # make point cloud
                cloud = []

                points = np.array(lidarData.point_cloud, dtype=np.dtype('f4'))
                points = np.reshape(points, (int(points.shape[0] / 3), 3))
                points = points * np.array([1, -1, -1])
                cloud = points.tolist()

                timeStamp = rospy.Time.now()
                
                groundtruth.data = labels.tolist()
                groundtruth.header.frame_id = frameID
                groundtruth.header.stamp = timeStamp

                pcloud.header.frame_id = frameID
                pcloud.header.stamp = timeStamp

                pcloud = pc2.create_cloud_xyz32(pcloud.header, cloud)
                
                #publish messages
                pub.publish(pcloud)
                pubGT.publish(groundtruth)

        # sleeps until next cycle 
        rate.sleep()



if __name__ == '__main__':
    try:
        rospy.init_node('airsim_lidar', anonymous=True)
        frameID = rospy.get_param('~frame_id', 'base_laser')
        pubNode =  rospy.get_param('~pub_node', 'airsim/lidar')
        pubNodeGT =  rospy.get_param('~pub_node_gt', 'airsim/lidargroundtruth')
        sensorName =  rospy.get_param('~sensor_name', 'lidar')
        vehicleName = rospy.get_param('~vehicle_name', 'airsimvehicle')
        lidar_airpub(frameID, pubNode, pubNodeGT, sensorName, vehicleName)
    except rospy.ROSInterruptException:
        pass
