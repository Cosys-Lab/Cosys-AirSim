#!/usr/bin/env python

import setup_path 
import airsim
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np


def camera_airpub(frameID, pubNode, cameraIndex, cameraType, cameraHeight, cameraWidth, cameraFrequency):
    pub = rospy.Publisher(pubNode, Image, queue_size=1)
    rospy.init_node('airsim_camera_pub', anonymous=True)
    rate = rospy.Rate(cameraFrequency) 

    # connect to the AirSim simulator 
    client = airsim.CarClient()
    client.confirmConnection()

    while not rospy.is_shutdown():

        # get camera images from the car
        if(cameraType == "Scene"):
            cameraTypeClass = airsim.ImageType.Scene
        elif(cameraType == "Segmentation"):
            cameraTypeClass = airsim.ImageType.Segmentation
        elif(cameraType == "DepthPerspective"):
            cameraTypeClass = airsim.ImageType.DepthPerspective
        elif(cameraType == "DepthVis"):
            cameraTypeClass = airsim.ImageType.DepthVis
        elif(cameraType == "Infrared"):
            cameraTypeClass = airsim.ImageType.Infrared
        elif(cameraType == "SurfaceNormals"):
            cameraTypeClass = airsim.ImageType.SurfaceNormals
        elif(cameraType == "DisparityNormalized"):
            cameraTypeClass = airsim.ImageType.DisparityNormalized
        else:
            cameraTypeClass = airsim.ImageType.Scene
            rospy.logwarn("Camera type not found, setting to Scene as default")
        responses = client.simGetImages([airsim.ImageRequest(cameraIndex, cameraTypeClass, False, False)])  #scene vision image in uncompressed RGB array

        for response in responses:
            img_rgb_string = response.image_data_uint8

        # Populate image message
        msg=Image() 
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frameID
        msg.encoding = "rgb8"
        msg.height = cameraHeight  # resolution should match values in settings.json 
        msg.width = cameraWidth
        msg.data = img_rgb_string
        msg.is_bigendian = 0
        msg.step = msg.width * 3

        # publish image message    
        pub.publish(msg)
        rospy.loginfo("Camera image published")

        # sleep until next cycle
        rate.sleep()



if __name__ == '__main__':
    try:
        topicName = rospy.get_param('~topic_name', 'camera')
        frameID = rospy.get_param('~frame_id', 'base_camera')
        cameraIndex =  rospy.get_param('~camera_index',"1")
        cameraType = rospy.get_param('~camera_type', 'Scene')
        cameraHeight = rospy.get_param('~camera_height', 360)
        cameraWidth = rospy.get_param('~camera_width', 640)
        cameraFrequency = rospy.get_param('~camera_frequency', 10)
        pubNode =  rospy.get_param('~pub_node', "airsim/" + topicName)
        camera_airpub(frameID, pubNode, cameraIndex, cameraType, cameraHeight, cameraWidth, cameraFrequency)
    except rospy.ROSInterruptException:
        pass
