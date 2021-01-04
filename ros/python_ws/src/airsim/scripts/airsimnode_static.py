#!/usr/bin/env python

import setup_path
import airsimpy
import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from airsim.msg import StringArray
import numpy as np
import os
import cv2
from cv_bridge import CvBridge
from multiprocessing import Queue
from airsimnode import handle_airsim_pose, handle_input_command, get_camera_type, is_pixels_as_float, get_image_bytes
import rosbag
import pyautogui

def airsim_pub(rosRate, rosIMURate, activeTuple, topicsTuple, framesTuple, cameraSettingsTuple,
                lidarName, echoName, imuName, vehicleName, transformPose, carcontrolInputTopic, publishAlternative, route_rosbag, dst_rosbag):

    # Reading from Tuples
    camera1Active = activeTuple[0]
    camera2Active = activeTuple[1]
    camera3Active = activeTuple[2]
    lidarActive = activeTuple[3]
    gpulidarActive = activeTuple[4]
    echoActive = activeTuple[5]
    imuActive = activeTuple[6]
    poseActive = activeTuple[7]
    carcontrolActive = activeTuple[8]

    sceneCamera1TopicName = topicsTuple[0]
    segmentationCamera1TopicName = topicsTuple[1]
    depthCamera1TopicName = topicsTuple[2]
    sceneCamera2TopicName = topicsTuple[3]
    segmentationCamera2TopicName = topicsTuple[4]
    depthCamera2TopicName = topicsTuple[5]
    sceneCamera3TopicName = topicsTuple[6]
    segmentationCamera3TopicName = topicsTuple[7]
    depthCamera3TopicName = topicsTuple[8]
    lidarTopicName = topicsTuple[9]
    lidarGroundtruthTopicName = topicsTuple[10]
    echoTopicName = topicsTuple[11]
    imuTopicName = topicsTuple[12]
    imuAltTopicName = topicsTuple[13]
    poseTopicName = topicsTuple[14]
    poseAltTopicName = topicsTuple[15]
    sceneCamera1MonoTopicName = topicsTuple[16]
    sceneCamera2MonoTopicName = topicsTuple[17]
    sceneCamera3MonoTopicName = topicsTuple[18]

    camera1Frame = framesTuple[0]
    camera2Frame = framesTuple[1]
    camera3Frame = framesTuple[2]
    lidarFrame = framesTuple[3]
    echoFrame = framesTuple[4]
    imuFrame = framesTuple[5]
    poseFrame = framesTuple[6]

    camera1Name = cameraSettingsTuple[0]
    camera2Name = cameraSettingsTuple[1]
    camera3Name = cameraSettingsTuple[2]
    camera1SceneOnly = cameraSettingsTuple[3]
    camera2SceneOnly = cameraSettingsTuple[4]
    camera3SceneOnly = cameraSettingsTuple[5]
    camera1Mono = cameraSettingsTuple[6]
    camera2Mono = cameraSettingsTuple[7]
    camera3Mono = cameraSettingsTuple[8]
    cameraHeight = cameraSettingsTuple[9]
    cameraWidth = cameraSettingsTuple[10]
    cameraSegmentationDisabled = cameraSettingsTuple[11]

    print("Connecting...")
    client = airsimpy.CarClient()
    client.confirmConnection(rospy.get_name())
    print("Connected")

    #should not need rate
    #rate = rospy.Rate(rosIMURate)

    route = rosbag.Bag(route_rosbag)
    output = rosbag.Bag(dst_rosbag, 'w')

    # Some temporary variables
    lastLidarTimeStamp = None
    lastEchoTimeStamp = None

    bridge = CvBridge()

    # Set up the requests for camera images
    requests = []
    #every element in the responseLocations array represent the location of the response of the respecitive camera and configuration
    #[camera1_scene, camaera1_segmentation, camera_1_depthplanner, camera2_...]
    responseLocations = [-1,-1,-1,-1,-1,-1,-1,-1,-1]
    tmp_i = 0
    if camera1Active:
        responseLocations[0] = tmp_i; tmp_i += 1
        requests.append(airsimpy.ImageRequest(camera1Name, get_camera_type("Scene"), is_pixels_as_float("Scene"), False))
        print("Cam 1 scene")
        if not camera1SceneOnly:
            if not cameraSegmentationDisabled:
                responseLocations[1] = tmp_i; tmp_i += 1
                requests.append(airsimpy.ImageRequest(camera1Name, get_camera_type("Segmentation"), is_pixels_as_float("Segmentation"), False))
                print("Cam 1 seg")
            responseLocations[2] = tmp_i; tmp_i += 1
            requests.append(airsimpy.ImageRequest(camera1Name, get_camera_type("DepthPlanner"), is_pixels_as_float("DepthPlanner"), False))
            print("Cam 1 depth")
    if camera2Active:
        responseLocations[3] = tmp_i; tmp_i += 1
        requests.append(airsimpy.ImageRequest(camera2Name, get_camera_type("Scene"), is_pixels_as_float("Scene"), False))
        print("Cam 2 scene")
        if not camera2SceneOnly:
            if not cameraSegmentationDisabled:
                responseLocations[4] = tmp_i; tmp_i += 1
                requests.append(airsimpy.ImageRequest(camera2Name, get_camera_type("Segmentation"), is_pixels_as_float("Segmentation"), False))
            responseLocations[5] = tmp_i; tmp_i += 1
            requests.append(airsimpy.ImageRequest(camera2Name, get_camera_type("DepthPlanner"), is_pixels_as_float("DepthPlanner"), False))
    if camera3Active:
        responseLocations[6] = tmp_i; tmp_i += 1
        requests.append(airsimpy.ImageRequest(camera3Name, get_camera_type("Scene"), is_pixels_as_float("Scene"), False))
        if not camera3SceneOnly:
            if not cameraSegmentationDisabled:
                responseLocations[7] = tmp_i; tmp_i += 1
                requests.append(airsimpy.ImageRequest(camera3Name, get_camera_type("Segmentation"), is_pixels_as_float("Segmentation"), False))
            responseLocations[8] = tmp_i; tmp_i += 1
            requests.append(airsimpy.ImageRequest(camera3Name, get_camera_type("DepthPlanner"), is_pixels_as_float("DepthPlanner"), False))

    print("Starting...: {}".format(requests))
    rospy.logwarn("Ensure focus is on the screen of simulator to allow auto configuration!")
    # for each pose in route generate measurements (camera, lidar, ...)
    #print(route.read_messages())
    rospy.logdebug(str(route.get_type_and_topic_info()))

    # Variables to automatically push for next configuration of environment
    initialPose = None
    hystSetNextConfiguration = True #start is ref position thus do not trigger next
    hystSetNextConfigurationSize = 0.5 #size of detection area in m
    print("Size of configuration detection area is {} m.".format(hystSetNextConfigurationSize))

    periodFrames = round(float(rosIMURate)/float(rosRate))
    print("Using frame period of {} to generate data.".format(periodFrames))
    currentFrame = 0
    for _, msg, t in route.read_messages(topics=poseTopicName):
        if currentFrame == periodFrames:
            currentFrame = 0
        else:
            currentFrame += 1
            continue
        
        rostimestamp = t
        timeStamp = msg.header.stamp
        position = airsimpy.Vector3r(msg.pose.position.x, -msg.pose.position.y, -msg.pose.position.z)
        orientation = airsimpy.Quaternionr(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w).inverse()
        orientation = airsimpy.Quaternionr(float(orientation.x_val), float(orientation.y_val), float(orientation.z_val), float(orientation.w_val))
        client.simSetVehiclePose(airsimpy.Pose(position, orientation), True, vehicleName)
        rospy.logdebug("Set pose at {}".format(str(t)))

        if initialPose is None:
            initialPose = position
        distanceToInitialPose = initialPose.distance_to(position)
        if distanceToInitialPose < hystSetNextConfigurationSize and not hystSetNextConfiguration:
            hystSetNextConfiguration = True
            print("Triggering next configuration...")
            pyautogui.press('o')
        elif distanceToInitialPose >= hystSetNextConfigurationSize and hystSetNextConfiguration:
            print("Waiting for trigger location of next configuration...")
            hystSetNextConfiguration = False

        cameraTimeStamp = timeStamp

        responses = client.simGetImages(requests)

        if camera1Active:
            if not camera1SceneOnly:
                img_rgb_string1 = get_image_bytes(responses[responseLocations[0]], "Scene")
                if not cameraSegmentationDisabled:
                    img_rgb_string2 = get_image_bytes(responses[responseLocations[1]], "Segmentation")
                img_rgb_string3 = get_image_bytes(responses[responseLocations[2]], "DepthPlanner")
            else:
                img_rgb_string1 = get_image_bytes(responses[responseLocations[0]], "Scene")

            if(len(img_rgb_string1) > 1):
                msg = Image()
                msg.height = cameraHeight
                msg.width = cameraWidth
                msg.is_bigendian = 0
                msg.encoding = "rgb8"
                msg.step = cameraWidth * 3
                msg.data = img_rgb_string1
                msg.header.stamp = cameraTimeStamp
                msg.header.frame_id = camera1Frame
                output.write(sceneCamera1TopicName, msg, t=rostimestamp)
                if camera1Mono:
                    img_rgb_string1_matrix = np.fromstring(img_rgb_string1, dtype=np.uint8).reshape(cameraHeight, cameraWidth, 3)
                    msg = bridge.cv2_to_imgmsg(cv2.cvtColor(img_rgb_string1_matrix, cv2.COLOR_RGB2GRAY), encoding="mono8")
                    msg.header.stamp = cameraTimeStamp
                    msg.header.frame_id = camera1Frame
                    output.write(sceneCamera1MonoTopicName, msg, t=rostimestamp)
                if not camera1SceneOnly:
                    if not cameraSegmentationDisabled:
                        msg.step = cameraWidth * 3
                        msg.data = img_rgb_string2
                        output.write(segmentationCamera1TopicName, msg, t=rostimestamp)
                    msg.encoding = "32FC1"
                    msg.step = cameraWidth * 4
                    msg.data = img_rgb_string3
                    output.write(depthCamera1TopicName, msg, t=rostimestamp)

        if camera2Active:
            if not camera2SceneOnly:
                img_rgb_string1 = get_image_bytes(responses[responseLocations[3]], "Scene")
                if not cameraSegmentationDisabled:
                    img_rgb_string2 = get_image_bytes(responses[responseLocations[4]], "Segmentation")
                img_rgb_string3 = get_image_bytes(responses[responseLocations[5]], "DepthPlanner")
            else:
                img_rgb_string1 = get_image_bytes(responses[responseLocations[3]], "Scene")

            if(len(img_rgb_string1) > 1):
                msg = Image()
                msg.height = cameraHeight
                msg.width = cameraWidth
                msg.is_bigendian = 0
                msg.encoding = "rgb8"
                msg.step = cameraWidth * 3
                msg.data = img_rgb_string1
                msg.header.stamp = cameraTimeStamp
                msg.header.frame_id = camera2Frame
                output.write(sceneCamera2TopicName, msg, t=rostimestamp)
                if camera2Mono:
                    img_rgb_string1_matrix = np.fromstring(img_rgb_string1, dtype=np.uint8).reshape(cameraHeight, cameraWidth, 3)
                    msg = bridge.cv2_to_imgmsg(cv2.cvtColor(img_rgb_string1_matrix, cv2.COLOR_RGB2GRAY), encoding="mono8")
                    msg.header.stamp = cameraTimeStamp
                    msg.header.frame_id = camera2Frame
                    output.write(sceneCamera2MonoTopicName, msg, t=rostimestamp)
                if not camera2SceneOnly:
                    if not cameraSegmentationDisabled:
                        msg.step = cameraWidth * 3
                        msg.data = img_rgb_string2
                        output.write(segmentationCamera2TopicName, msg, t=rostimestamp)
                    msg.encoding = "32FC1"
                    msg.step = cameraWidth * 4
                    msg.data = img_rgb_string3
                    output.write(depthCamera2TopicName, msg, t=rostimestamp)

        if camera3Active:
            if not camera3SceneOnly:
                img_rgb_string1 = get_image_bytes(responses[responseLocations[6]], "Scene")
                if not cameraSegmentationDisabled:
                    img_rgb_string2 = get_image_bytes(responses[responseLocations[7]], "Segmentation")
                img_rgb_string3 = get_image_bytes(responses[responseLocations[8]], "DepthPlanner")
            else:
                img_rgb_string1 = get_image_bytes(responses[responseLocations[6]], "Scene")

            if(len(img_rgb_string1) > 1):
                msg = Image()
                msg.height = cameraHeight
                msg.width = cameraWidth
                msg.is_bigendian = 0
                msg.encoding = "rgb8"
                msg.step = cameraWidth * 3
                msg.data = img_rgb_string1
                msg.header.stamp = cameraTimeStamp
                msg.header.frame_id = camera3Frame
                output.write(sceneCamera3TopicName, msg, t=rostimestamp)
                if camera3Mono:
                    img_rgb_string1_matrix = np.fromstring(img_rgb_string1, dtype=np.uint8).reshape(cameraHeight, cameraWidth, 3)
                    msg = bridge.cv2_to_imgmsg(cv2.cvtColor(img_rgb_string1_matrix, cv2.COLOR_RGB2GRAY), encoding="mono8")
                    msg.header.stamp = cameraTimeStamp
                    msg.header.frame_id = camera3Frame
                    output.write(sceneCamera3MonoTopicName, msg, t=rostimestamp)
                if not camera3SceneOnly:
                    if not cameraSegmentationDisabled:
                        msg.step = cameraWidth * 3
                        msg.data = img_rgb_string2
                        output.write(segmentationCamera3TopicName, msg, t=rostimestamp)
                    msg.encoding = "32FC1"
                    msg.step = cameraWidth * 4
                    msg.data = img_rgb_string3
                    output.write(depthCamera3TopicName, msg, t=rostimestamp)

        if lidarActive or gpulidarActive:
            # get lidar data
            if lidarActive:
                lidarData = client.getLidarData(lidarName, vehicleName)
            if gpulidarActive:
                lidarData = client.getGPULidarData(lidarName, vehicleName)

            if lidarData.time_stamp != lastLidarTimeStamp:
                # Check if there are any points in the data
                if (len(lidarData.point_cloud) < 4):
                    lastLidarTimeStamp = timeStamp
                else:
                    lastLidarTimeStamp = timeStamp

                    # initiate point cloud
                    pcloud = PointCloud2()
                    groundtruth = StringArray()

                    labels = np.array(lidarData.groundtruth, dtype=np.dtype('U'))
                    points = np.array(lidarData.point_cloud, dtype=np.dtype('f4'))
                    points = np.reshape(points, (int(points.shape[0] / 3), 3))
                    points = points * np.array([1, -1, -1])
                    cloud = points.tolist()
                    groundtruth.data = labels.tolist()
                    groundtruth.header.frame_id = lidarFrame
                    groundtruth.header.stamp = timeStamp
                    pcloud.header.frame_id = lidarFrame
                    pcloud.header.stamp = timeStamp
                    pcloud = pc2.create_cloud_xyz32(pcloud.header, cloud)

                    # publish messages
                    output.write(lidarTopicName, pcloud, t=rostimestamp)
                    output.write(lidarGroundtruthTopicName, groundtruth, t=rostimestamp)

        if echoActive:
            # get echo data
            echoData = client.getEchoData(echoName, vehicleName)

            if echoData.time_stamp != lastEchoTimeStamp:
                # Check if there are any points in the data
                if (len(echoData.point_cloud) < 4):
                    lastEchoTimeStamp = timeStamp
                else:
                    lastEchoTimeStamp = timeStamp

                    # initiate point cloud
                    pcloud = PointCloud2()

                    points = np.array(echoData.point_cloud, dtype=np.dtype('f4'))
                    points = np.reshape(points, (int(points.shape[0] / 5), 5))
                    points = points * np.array([1, -1, -1, 1, 1])
                    cloud = points.tolist()
                    pcloud.header.frame_id = echoFrame
                    pcloud.header.stamp = timeStamp
                    fields = [
                        PointField('x', 0, PointField.FLOAT32, 1),
                        PointField('y', 4, PointField.FLOAT32, 1),
                        PointField('z', 8, PointField.FLOAT32, 1),
                        PointField('a', 12, PointField.FLOAT32, 1),
                        PointField('d', 16, PointField.FLOAT32, 1)
                    ]
                    pcloud = pc2.create_cloud(pcloud.header, fields, cloud)

                    # publish messages
                    output.write(echoTopicName, pcloud, t=rostimestamp)

    #copy all messages of route bag to output
    for topic, msg, t in route.read_messages():
        output.write(topic, msg, t)
    output.close()


if __name__ == '__main__':
    try:
        #should not need a publisher?
        rospy.init_node('airsim_publisher', anonymous=True)

        # Desired update frequencys
        rosRate = rospy.get_param('~rate', 10)
        rosIMURate = rospy.get_param('~imu_rate', 100)

        # Active publish topics
        camera1Active = rospy.get_param('~camera1_active', 0)
        camera2Active = rospy.get_param('~camera2_active', 0)
        camera3Active = rospy.get_param('~camera3_active', 0)
        lidarActive = rospy.get_param('~lidar_active', 0)
        gpulidarActive = rospy.get_param('~gpulidar_active', 0)
        echoActive = rospy.get_param('~echo_active', 0)
        imuActive = rospy.get_param('~imu_active', 0)
        poseActive = rospy.get_param('~pose_active', 1)
        carcontrolActive = rospy.get_param('~carcontrol_active', 0)
        activeTuple = (camera1Active, camera2Active, camera3Active, lidarActive, gpulidarActive, echoActive, imuActive, poseActive, carcontrolActive)

        # Publish topic names
        sceneCamera1TopicName = rospy.get_param('~topic_scene_camera1', 'airsim/rgb/image')
        segmentationCamera1TopicName = rospy.get_param('~topic_segmentation_camera1', 'airsim/segmentation/image')
        depthCamera1TopicName = rospy.get_param('~topic_depth_camera1', 'airsim/depth/image')
        sceneCamera2TopicName = rospy.get_param('~topic_scene_camera2', 'airsim/rgb2/image')
        segmentationCamera2TopicName = rospy.get_param('~topic_segmentation_camera2', 'airsim/segmentation2/image')
        depthCamera2TopicName = rospy.get_param('~topic_depth_camera2', 'airsim/depth2/image')
        sceneCamera3TopicName = rospy.get_param('~topic_scene_camera3', 'airsim/rgb3/image')
        segmentationCamera3TopicName = rospy.get_param('~topic_segmentation_camera3', 'airsim/segmentation3/image')
        depthCamera3TopicName = rospy.get_param('~topic_depth_camera3', 'airsim/depth3/image')
        lidarTopicName = rospy.get_param('~topic_lidar', 'airsim/lidar')
        lidarGroundtruthTopicName =  rospy.get_param('~topic_lidar_groundtruth', 'airsim/lidargroundtruth')
        echoTopicName = rospy.get_param('~topic_echo', 'airsim/echo')
        imuTopicName = rospy.get_param('~topic_imu', 'airsim/imu')
        imuAltTopicName = rospy.get_param('~topic_imu_alt', 'imualt')
        poseTopicName = rospy.get_param('~topic_pose', 'airsim/gtpose')
        poseAltTopicName = rospy.get_param('~topic_pose_alt', 'airsim/gtposealt')
        sceneCamera1MonoTopicName = rospy.get_param('~topic_scene_camera1_mono', 'airsim/mono/image')
        sceneCamera2MonoTopicName = rospy.get_param('~topic_scene_camera2_mono', 'airsim/mono2/image')
        sceneCamera3MonoTopicName = rospy.get_param('~topic_scene_camera3_mono', 'airsim/mono3/image')
        topicsTuple = (sceneCamera1TopicName, segmentationCamera1TopicName, depthCamera1TopicName, sceneCamera2TopicName, segmentationCamera2TopicName, depthCamera2TopicName, sceneCamera3TopicName, segmentationCamera3TopicName, depthCamera3TopicName,
                       lidarTopicName, lidarGroundtruthTopicName, echoTopicName, imuTopicName, imuAltTopicName, poseTopicName, poseAltTopicName,
                       sceneCamera1MonoTopicName, sceneCamera2MonoTopicName, sceneCamera3MonoTopicName)

        # Frames
        camera1Frame = rospy.get_param('~camera1_frame_id', 'base_camera')
        camera2Frame = rospy.get_param('~camera2_frame_id', 'base_camera')
        camera3Frame = rospy.get_param('~camera3_frame_id', 'base_camera')
        lidarFrame = rospy.get_param('~lidar_frame_id', 'base_laser')
        echoFrame = rospy.get_param('~echo_frame_id', 'base_echo')
        imuFrame = rospy.get_param('~imu_frame_id', 'base_link')
        poseFrame = rospy.get_param('~pose_frame_id', 'world')
        framesTuple = (camera1Frame, camera2Frame, camera3Frame, lidarFrame, echoFrame, imuFrame, poseFrame )

        # Camera settings
        camera1Name = rospy.get_param('~camera1_name', "front_center")
        camera2Name = rospy.get_param('~camera2_name', "front_left")
        camera3Name = rospy.get_param('~camera3_name', "front_right")
        camera1SceneOnly = rospy.get_param('~camera1_sceneonly', 0)
        camera2SceneOnly = rospy.get_param('~camera2_sceneonly', 0)
        camera3SceneOnly = rospy.get_param('~camera3_sceneonly', 0)
        camera1Mono = rospy.get_param('~camera1_mono', 0)
        camera2Mono = rospy.get_param('~camera2_mono', 0)
        camera3Mono = rospy.get_param('~camera3_mono', 0)
        cameraHeight = rospy.get_param('~camera_height', 540)
        cameraWidth = rospy.get_param('~camera_width', 960)
        cameraSegementationDisabled = rospy.get_param('~camera_segmentation_disabled', 0)
        cameraSettingsTuple = (camera1Name, camera2Name, camera3Name, camera1SceneOnly, camera2SceneOnly, camera3SceneOnly, camera1Mono, camera2Mono, camera3Mono, cameraHeight, cameraWidth, cameraSegementationDisabled)

        # Lidar settings
        lidarName =  rospy.get_param('~lidar_name', 'lidar')

        # Echo settings
        echoName =  rospy.get_param('~echo_name', 'echo')

        # IMU settings
        imuName =  rospy.get_param('~imu_name', 'imu')

        # Vehicle settings
        vehicleName = rospy.get_param('~vehicle_name', 'airsimvehicle')

        # Transform the groundtruth pose to the imu frame
        # Best to turn this off to not intervene with SLAM systems
        transformPose = rospy.get_param('~pose_transform', 0)

        # Car Control settings
        carcontrolInputTopic = rospy.get_param('~carcontrol_input_topic', 'cmd_vel')

        # Publish on alternative coordinate frame settings with HySLAM
        publishAlternative = rospy.get_param('~publish_alternative', 0)
        route_rosbag = rospy.get_param('~route_rosbag', "airsim_hyslam_route_start.bag")
        dst_rosbag = rospy.get_param('~dst_rosbag', "airsim_hyslam_benchmark_test.bag")

        airsim_pub(rosRate, rosIMURate, activeTuple, topicsTuple, framesTuple, cameraSettingsTuple, lidarName, echoName, imuName,
                   vehicleName, transformPose, carcontrolInputTopic, publishAlternative, route_rosbag, dst_rosbag)

    except rospy.ROSInterruptException:
        pass
