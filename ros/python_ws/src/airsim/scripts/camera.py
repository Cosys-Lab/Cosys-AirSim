#!/usr/bin/env python

import setup_path 
import airsimpy
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import os

def get_camera_type(cameraType):
    if(cameraType == "Scene"):
        cameraTypeClass = airsimpy.ImageType.Scene
    elif(cameraType == "Segmentation"):
        cameraTypeClass = airsimpy.ImageType.Segmentation
    elif(cameraType == "DepthPerspective"):
        cameraTypeClass = airsimpy.ImageType.DepthPerspective
    elif(cameraType == "DepthPlanner"):
        cameraTypeClass = airsimpy.ImageType.DepthPlanner
    elif(cameraType == "DepthVis"):
        cameraTypeClass = airsimpy.ImageType.DepthVis
    elif(cameraType == "Infrared"):
        cameraTypeClass = airsimpy.ImageType.Infrared
    elif(cameraType == "SurfaceNormals"):
        cameraTypeClass = airsimpy.ImageType.SurfaceNormals
    elif(cameraType == "DisparityNormalized"):
        cameraTypeClass = airsimpy.ImageType.DisparityNormalized
    else:
        cameraTypeClass = airsimpy.ImageType.Scene
        rospy.logwarn("Camera type %s not found, setting to Scene as default", cameraType)
    return cameraTypeClass


def is_pixels_as_float(cameraType):
    if(cameraType == "Scene"):
        return False
    elif(cameraType == "Segmentation"):
       return False
    elif(cameraType == "DepthPerspective"):
        return True
    elif(cameraType == "DepthPlanner"):
        return True
    elif(cameraType == "DepthVis"):
        return True
    elif(cameraType == "Infrared"):
        return False
    elif(cameraType == "SurfaceNormals"):
        return False
    elif(cameraType == "DisparityNormalized"):
        return True
    else:
        return False


def get_image_bytes(data, cameraType, MaxDepthDistance):
    if(cameraType == "Scene"):
        img_rgb_string = data.image_data_uint8
    elif(cameraType == "Segmentation"):
        img_rgb_string = data.image_data_uint8
    elif(cameraType == "DepthPerspective"):
        img_rgb_float = data.image_data_float
        img_rgb_float = np.asarray(img_rgb_float)
        img_rgb_float[img_rgb_float > MaxDepthDistance] = MaxDepthDistance
        img_rgb_int = np.rint((img_rgb_float / MaxDepthDistance) * 255)
        img_rgb_int = np.repeat(img_rgb_int, 3).astype(dtype=np.uint8)
        img_rgb_string = img_rgb_int.tobytes()
    elif(cameraType == "DepthPlanner"):
        img_rgb_float = data.image_data_float
        img_rgb_float = np.asarray(img_rgb_float)
        img_rgb_float[img_rgb_float > MaxDepthDistance] = MaxDepthDistance
        img_rgb_int = np.rint((img_rgb_float / MaxDepthDistance) * 255)
        img_rgb_int = np.repeat(img_rgb_int, 3).astype(dtype=np.uint8)
        img_rgb_string = img_rgb_int.tobytes()
    elif(cameraType == "DepthVis"):
        img_rgb_float = data.image_data_float
        img_rgb_float = np.asarray(img_rgb_float)
        img_rgb_int = np.rint((img_rgb_float / np.max(img_rgb_float)) * 255)
        img_rgb_int = np.repeat(img_rgb_int, 3).astype(dtype=np.uint8)
        img_rgb_string = img_rgb_int.tobytes()
    elif(cameraType == "Infrared"):
        img_rgb_string = data.image_data_uint8
    elif(cameraType == "SurfaceNormals"):
        img_rgb_string = data.image_data_uint8
    elif(cameraType == "DisparityNormalized"):
        img_rgb_float = data.image_data_float
        img_rgb_float = np.asarray(img_rgb_float)
        img_rgb_int = np.rint((img_rgb_float / np.max(img_rgb_float)) * 255)
        img_rgb_int = np.repeat(img_rgb_int, 3).astype(dtype=np.uint8)
        img_rgb_string = img_rgb_int.tobytes()
    else:
        img_rgb_string = data.image_data_uint8
    return img_rgb_string


def camera_airpub(frameID, pubNode1, pubNode2, pubNode3, camera2Active, camera3Active, cameraIndex1, cameraIndex2, cameraIndex3, cameraType1, cameraType2, cameraType3, cameraHeight, cameraWidth, cameraFrequency, MaxDepthDistance):
    pub1 = rospy.Publisher(pubNode1, Image, queue_size=1)
    if camera2Active:
        pub2 = rospy.Publisher(pubNode2, Image, queue_size=1)
    if camera3Active:
        pub3 = rospy.Publisher(pubNode3, Image, queue_size=1)    
    rate = rospy.Rate(cameraFrequency) 

    # connect to the AirSim simulator 
    client = airsimpy.CarClient()
    
    client.confirmConnection(rospy.get_name())
    index = 0
    while not rospy.is_shutdown():

        if camera2Active == 0 and camera3Active == 0:
            responses = client.simGetImages([airsimpy.ImageRequest(cameraIndex1, get_camera_type(cameraType1), is_pixels_as_float(cameraType1), False)])
            
            img_rgb_string1 = get_image_bytes(responses[0], cameraType1, MaxDepthDistance)

            msg=Image() 
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = frameID
            msg.encoding = "rgb8"
            msg.height = cameraHeight 
            msg.width = cameraWidth
            msg.data = img_rgb_string1
            msg.is_bigendian = 0
            msg.step = msg.width * 3

            pub1.publish(msg)

        elif camera2Active == 1 and camera3Active == 0:
            responses = client.simGetImages([
                airsimpy.ImageRequest(cameraIndex1, get_camera_type(cameraType1), is_pixels_as_float(cameraType1), False),
                airsimpy.ImageRequest(cameraIndex2, get_camera_type(cameraType2), is_pixels_as_float(cameraType2), False)]) 

            img_rgb_string1 = get_image_bytes(responses[0], cameraType1, MaxDepthDistance)
            img_rgb_string2 = get_image_bytes(responses[1], cameraType2, MaxDepthDistance)

            msg=Image() 
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = frameID
            msg.encoding = "rgb8"
            msg.height = cameraHeight 
            msg.width = cameraWidth
            msg.data = img_rgb_string1
            msg.is_bigendian = 0
            msg.step = msg.width * 3

            pub1.publish(msg)
            msg.data = img_rgb_string2
            pub2.publish(msg)

        elif camera2Active == 0 and camera3Active == 1:
            responses = client.simGetImages([
                airsimpy.ImageRequest(cameraIndex1, get_camera_type(cameraType1), is_pixels_as_float(cameraType1), False),
                airsimpy.ImageRequest(cameraIndex3, get_camera_type(cameraType3), is_pixels_as_float(cameraType3), False)]) 
            
            img_rgb_string1 = get_image_bytes(responses[0], cameraType1, MaxDepthDistance)
            img_rgb_string3 = get_image_bytes(responses[1], cameraType3, MaxDepthDistance)

            msg=Image() 
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = frameID
            msg.encoding = "rgb8"
            msg.height = cameraHeight 
            msg.width = cameraWidth
            msg.data = img_rgb_string1
            msg.is_bigendian = 0
            msg.step = msg.width * 3

            pub1.publish(msg)
            msg.data = img_rgb_string3
            pub3.publish(msg)

        elif camera2Active == 1 and camera3Active == 1:    
            responses = client.simGetImages([
                airsimpy.ImageRequest(cameraIndex1, get_camera_type(cameraType1), is_pixels_as_float(cameraType1), False),
                airsimpy.ImageRequest(cameraIndex2, get_camera_type(cameraType2), is_pixels_as_float(cameraType2), False),
                airsimpy.ImageRequest(cameraIndex3, get_camera_type(cameraType3), is_pixels_as_float(cameraType3), False)]) 

            img_rgb_string1 = get_image_bytes(responses[0], cameraType1, MaxDepthDistance)
            img_rgb_string2 = get_image_bytes(responses[1], cameraType2, MaxDepthDistance)
            img_rgb_string3 = get_image_bytes(responses[2], cameraType3, MaxDepthDistance)

            msg=Image() 
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = frameID
            msg.encoding = "rgb8"
            msg.height = cameraHeight 
            msg.width = cameraWidth
            msg.data = img_rgb_string1
            msg.is_bigendian = 0
            msg.step = msg.width * 3
            pub1.publish(msg)
            msg.data = img_rgb_string2
            pub2.publish(msg)
            msg.data = img_rgb_string3
            pub3.publish(msg)

        # sleep until next cycle
        rate.sleep()
        index = index + 1



if __name__ == '__main__':
    try:
        rospy.init_node('airsim_camera', anonymous=True)
        topicName1 = rospy.get_param('~topic_name_1', 'airsim/camera/scene')
        topicName2 = rospy.get_param('~topic_name_2', 'airsim/camera/segmentation')
        topicName3 = rospy.get_param('~topic_name_3', 'airsim/camera/depth')
        frameID = rospy.get_param('~frame_id', 'base_camera')
        camera2Active =  rospy.get_param('~camera_2_active', 1)
        camera3Active =  rospy.get_param('~camera_3_active', 1)
        cameraIndex1 =  rospy.get_param('~camera_index_1', "front_center")
        cameraIndex2 =  rospy.get_param('~camera_index_2', "front_center")
        cameraIndex3 =  rospy.get_param('~camera_index_3', "front_center")
        cameraType1 = rospy.get_param('~camera_type_1', 'Scene')
        cameraType2 = rospy.get_param('~camera_type_2', 'Segmentation')
        cameraType3 = rospy.get_param('~camera_type_3', 'DepthPlanner')
        cameraHeight = rospy.get_param('~camera_height', 540)
        cameraWidth = rospy.get_param('~camera_width', 960)
        cameraFrequency = rospy.get_param('~camera_frequency', 10)
        MaxDepthDistance = rospy.get_param('~max_depth_distance', 10)
        pubNode1 =  rospy.get_param('~pub_node1', topicName1)
        pubNode2 =  rospy.get_param('~pub_node2', topicName2)
        pubNode3 =  rospy.get_param('~pub_node3', topicName3)
        camera_airpub(frameID, pubNode1, pubNode2, pubNode3, camera2Active, camera3Active,
                      cameraIndex1, cameraIndex2, cameraIndex3, cameraType1, cameraType2, cameraType3,
                      cameraHeight, cameraWidth, cameraFrequency, MaxDepthDistance)
    except rospy.ROSInterruptException:
        pass
