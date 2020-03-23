#!/usr/bin/env python

import setup_path
import airsimpy
import rospy
import tf2_ros
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, TransformStamped
from geometry_msgs.msg import Twist
from airsim.msg import StringArray
import numpy as np
import os

def handle_airsim_pose(msg, args):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

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

def handle_input_command(msg, args):
    # set the controls for car
    car_controls = airsimpy.CarControls()
    if (msg.linear.x < 0):
        car_controls.is_manual_gear = True
        car_controls.manual_gear = -1
    else:
        car_controls.is_manual_gear = False
    car_controls.throttle = msg.linear.x
    car_controls.steering = -msg.angular.z
    args[1].setCarControls(car_controls, args[0])
    print("now")

def get_camera_type(cameraType):
    if (cameraType == "Scene"):
        cameraTypeClass = airsimpy.ImageType.Scene
    elif (cameraType == "Segmentation"):
        cameraTypeClass = airsimpy.ImageType.Segmentation
    elif (cameraType == "DepthPerspective"):
        cameraTypeClass = airsimpy.ImageType.DepthPerspective
    elif (cameraType == "DepthPlanner"):
        cameraTypeClass = airsimpy.ImageType.DepthPlanner
    elif (cameraType == "DepthVis"):
        cameraTypeClass = airsimpy.ImageType.DepthVis
    elif (cameraType == "Infrared"):
        cameraTypeClass = airsimpy.ImageType.Infrared
    elif (cameraType == "SurfaceNormals"):
        cameraTypeClass = airsimpy.ImageType.SurfaceNormals
    elif (cameraType == "DisparityNormalized"):
        cameraTypeClass = airsimpy.ImageType.DisparityNormalized
    else:
        cameraTypeClass = airsimpy.ImageType.Scene
        rospy.logwarn("Camera type %s not found, setting to Scene as default", cameraType)
    return cameraTypeClass


def is_pixels_as_float(cameraType):
    if (cameraType == "Scene"):
        return False
    elif (cameraType == "Segmentation"):
        return False
    elif (cameraType == "DepthPerspective"):
        return True
    elif (cameraType == "DepthPlanner"):
        return True
    elif (cameraType == "DepthVis"):
        return True
    elif (cameraType == "Infrared"):
        return False
    elif (cameraType == "SurfaceNormals"):
        return False
    elif (cameraType == "DisparityNormalized"):
        return True
    else:
        return False


def get_image_bytes(data, cameraType, MaxDepthDistance):
    if (cameraType == "Scene"):
        img_rgb_string = data.image_data_uint8
    elif (cameraType == "Segmentation"):
        img_rgb_string = data.image_data_uint8
    elif (cameraType == "DepthPerspective"):
        img_rgb_float = data.image_data_float
        img_rgb_float = np.asarray(img_rgb_float)
        img_rgb_float[img_rgb_float > MaxDepthDistance] = MaxDepthDistance
        img_rgb_int = np.rint((img_rgb_float / MaxDepthDistance) * 255)
        img_rgb_int = np.repeat(img_rgb_int, 3).astype(dtype=np.uint8)
        img_rgb_string = img_rgb_int.tobytes()
    elif (cameraType == "DepthPlanner"):
        img_rgb_float = data.image_data_float
        img_rgb_float = np.asarray(img_rgb_float)
        img_rgb_float[img_rgb_float > MaxDepthDistance] = MaxDepthDistance
        img_rgb_int = np.rint((img_rgb_float / MaxDepthDistance) * 255)
        img_rgb_int = np.repeat(img_rgb_int, 3).astype(dtype=np.uint8)
        img_rgb_string = img_rgb_int.tobytes()
    elif (cameraType == "DepthVis"):
        img_rgb_float = data.image_data_float
        img_rgb_float = np.asarray(img_rgb_float)
        img_rgb_int = np.rint((img_rgb_float / np.max(img_rgb_float)) * 255)
        img_rgb_int = np.repeat(img_rgb_int, 3).astype(dtype=np.uint8)
        img_rgb_string = img_rgb_int.tobytes()
    elif (cameraType == "Infrared"):
        img_rgb_string = data.image_data_uint8
    elif (cameraType == "SurfaceNormals"):
        img_rgb_string = data.image_data_uint8
    elif (cameraType == "DisparityNormalized"):
        img_rgb_float = data.image_data_float
        img_rgb_float = np.asarray(img_rgb_float)
        img_rgb_int = np.rint((img_rgb_float / np.max(img_rgb_float)) * 255)
        img_rgb_int = np.repeat(img_rgb_int, 3).astype(dtype=np.uint8)
        img_rgb_string = img_rgb_int.tobytes()
    else:
        img_rgb_string = data.image_data_uint8
    return img_rgb_string


def airsim_pub(activeTuple, topicsTuple, framesTuple, cameraSettingsTuple, lidarName, imuName,
                      vehicleName, carcontrolInputTopic):
    # Reading from Tuples
    cameraActive = activeTuple[0]
    lidarActive = activeTuple[1]
    gpulidarActive = activeTuple[2]
    imuActive = activeTuple[3]
    poseActive = activeTuple[4]
    carcontrolActive = activeTuple[5]

    sceneCameraTopicName = topicsTuple[0]
    segmentationCameraTopicName = topicsTuple[1]
    depthCameraTopicName = topicsTuple[2]
    lidarTopicName = topicsTuple[3]
    lidarGroundtruthTopicName = topicsTuple[4]
    imuTopicName = topicsTuple[5]
    poseTopicName = topicsTuple[6]

    cameraFrame = framesTuple[0]
    lidarFrame = framesTuple[1]
    imuFrame = framesTuple[2]
    poseFrame = framesTuple[3]

    cameraName = cameraSettingsTuple[0]
    cameraHeight = cameraSettingsTuple[1]
    cameraWidth = cameraSettingsTuple[2]
    maxDepthDistance = cameraSettingsTuple[3]

    client = airsimpy.CarClient()
    client.confirmConnection(rospy.get_name())

    rate = rospy.Rate(10)

    if cameraActive:
        sceneCameraPub = rospy.Publisher(sceneCameraTopicName, Image, queue_size=1)
        segmentationCameraPub = rospy.Publisher(segmentationCameraTopicName, Image, queue_size=1)
        depthCameraPub = rospy.Publisher(depthCameraTopicName, Image, queue_size=1)
    if lidarActive or gpulidarActive:
        lidarPub = rospy.Publisher(lidarTopicName, PointCloud2, queue_size=1)
    if lidarActive:
        lidarGroundtruthPub = rospy.Publisher(lidarGroundtruthTopicName, StringArray, queue_size=1)
    if imuActive:
        imuPub = rospy.Publisher(imuTopicName, Imu, queue_size=1)
    if poseActive:
        rospy.Subscriber(poseTopicName, PoseStamped, handle_airsim_pose, (poseFrame, imuFrame))
        posePub = rospy.Publisher(poseTopicName, PoseStamped, queue_size=1)
    if carcontrolActive:
        client.enableApiControl(True)
        rospy.Subscriber(carcontrolInputTopic, Twist, handle_input_command, (vehicleName, client))

    # Some temporary variables
    lastTimeStamp= None

    while not rospy.is_shutdown():
        if cameraActive:
            # Get camera images
            responses = client.simGetImages([
                airsimpy.ImageRequest(cameraName, get_camera_type("Scene"), is_pixels_as_float("Scene"),
                                      False),
                airsimpy.ImageRequest(cameraName, get_camera_type("Segmentation"), is_pixels_as_float("Segmentation"),
                                      False),
                airsimpy.ImageRequest(cameraName, get_camera_type("DepthPlanner"), is_pixels_as_float("DepthPlanner"),
                                      False)])

            img_rgb_string1 = get_image_bytes(responses[0], "Scene", maxDepthDistance)
            img_rgb_string2 = get_image_bytes(responses[1], "Segmentation", maxDepthDistance)
            img_rgb_string3 = get_image_bytes(responses[2], "DepthPlanner", maxDepthDistance)

            msg = Image()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = cameraFrame
            msg.encoding = "rgb8"
            msg.height = cameraHeight
            msg.width = cameraWidth
            msg.data = img_rgb_string1
            msg.is_bigendian = 0
            msg.step = msg.width * 3
            sceneCameraPub.publish(msg)
            msg.data = img_rgb_string2
            segmentationCameraPub.publish(msg)
            msg.data = img_rgb_string3
            depthCameraPub.publish(msg)

        if lidarActive or gpulidarActive:

            # initiate point cloud
            pcloud = PointCloud2()
            if lidarActive:
                groundtruth = StringArray()

            # get lidar data
            if lidarActive:
                lidarData = client.getLidarData(lidarName, vehicleName)
            if gpulidarActive:
                lidarData = client.getGPULidarData(lidarName, vehicleName)

            if lidarData.time_stamp != lastTimeStamp:
                # Check if there are any points in the data
                if (len(lidarData.point_cloud) < 4):
                    lastTimeStamp = lidarData.time_stamp
                else:
                    lastTimeStamp = lidarData.time_stamp
                    if lidarActive:
                        labels = np.array(lidarData.groundtruth, dtype=np.dtype('U'))
                    points = np.array(lidarData.point_cloud, dtype=np.dtype('f4'))
                    points = np.reshape(points, (int(points.shape[0] / 3), 3))
                    points = points * np.array([1, -1, -1])
                    cloud = points.tolist()
                    timeStamp = rospy.Time.now()
                    if lidarActive:
                        groundtruth.data = labels.tolist()
                        groundtruth.header.frame_id = lidarFrame
                        groundtruth.header.stamp = timeStamp
                    pcloud.header.frame_id = lidarFrame
                    pcloud.header.stamp = timeStamp
                    pcloud = pc2.create_cloud_xyz32(pcloud.header, cloud)

                    # publish messages
                    lidarPub.publish(pcloud)
                    if lidarActive:
                        lidarGroundtruthPub.publish(groundtruth)

        if imuActive:
            # get data of IMU sensor
            imuData = client.getImuData(imuName, vehicleName)

            # populate Imu ros message
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = imuFrame
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

            # publish Imu message
            imuPub.publish(imu_msg)

        if poseActive:
            # get state of the car
            pose = client.simGetVehiclePose(vehicleName)
            pos = pose.position
            orientation = pose.orientation.inverse()

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
            simPose.header.frame_id = poseFrame

            # publish PoseStamped message
            posePub.publish(simPose)

        # sleep until next cycle
        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('airsim_publisher', anonymous=True)

        # Active publish topics
        cameraActive = rospy.get_param('~camera_active', 1)
        lidarActive = rospy.get_param('~lidar_active', 1)
        gpulidarActive = rospy.get_param('~gpulidar_active', 0)
        imuActive = rospy.get_param('~imu_active', 1)
        poseActive = rospy.get_param('~pose_active', 1)
        carcontrolActive = rospy.get_param('~carcontrol_active', 0)
        activeTuple = (cameraActive, lidarActive, gpulidarActive, imuActive, poseActive, carcontrolActive)

        # Publish topic names
        sceneCameraTopicName = rospy.get_param('~topic_scene_camera', 'airsim/camera/scene')
        segmentationCameraTopicName = rospy.get_param('~topic_segmentation_camera', 'airsim/camera/segmentation')
        depthCameraTopicName = rospy.get_param('~topic_depth_camera', 'airsim/camera/depth')
        lidarTopicName = rospy.get_param('~topic_lidar', 'airsim/lidar')
        lidarGroundtruthTopicName =  rospy.get_param('~topic_lidar_groundtruth', 'airsim/lidargroundtruth')
        imuTopicName = rospy.get_param('~topic_imu', 'airsim/imu')
        poseTopicName = rospy.get_param('~topic_pose', 'airsim/pose')
        topicsTuple = (sceneCameraTopicName, segmentationCameraTopicName, depthCameraTopicName, lidarTopicName, lidarGroundtruthTopicName, imuTopicName, poseTopicName)

        # Frames
        cameraFrame = rospy.get_param('~camera_frame_id', 'base_camera')
        lidarFrame = rospy.get_param('~lidar_frame_id', 'base_laser')
        imuFrame = rospy.get_param('~imu_frame_id', 'base_link')
        poseFrame = rospy.get_param('~pose_frame_id', 'world')
        framesTuple = (cameraFrame, lidarFrame, imuFrame, poseFrame )

        # Camera settings
        cameraName = rospy.get_param('~camera_name', "front_center")
        cameraHeight = rospy.get_param('~camera_height', 540)
        cameraWidth = rospy.get_param('~camera_width', 960)
        maxDepthDistance = rospy.get_param('~camera_max_depth', 10)
        cameraSettingsTuple = (cameraName, cameraHeight, cameraWidth, maxDepthDistance)

        # Lidar settings
        lidarName =  rospy.get_param('~lidar_name', 'lidar')

        # IMU settings
        imuName =  rospy.get_param('~imu_name', 'imu')

        # Vehicle settings
        vehicleName = rospy.get_param('~vehicle_name', 'airsimvehicle')

        # Car Control settings
        carcontrolInputTopic = rospy.get_param('~carcontrol_input_topic', 'cmd_vel')

        airsim_pub(activeTuple, topicsTuple, framesTuple, cameraSettingsTuple, lidarName, imuName,
                      vehicleName, carcontrolInputTopic )

    except rospy.ROSInterruptException:
        pass
