#!/usr/bin/env python

import setup_path
import airsimpy
import rospy
import tf2_ros
import time
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
import cv2
from cv_bridge import CvBridge
from multiprocessing import Queue


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
    q = args
    try:
        q.put(msg)
    except:
        print("Input queue full")


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


def get_image_bytes(data, cameraType):
    if (cameraType == "Scene"):
        img_rgb_string = data.image_data_uint8
    elif (cameraType == "Segmentation"):
        img_rgb_string = data.image_data_uint8
    elif (cameraType == "DepthPerspective"):
        img_depth_float = data.image_data_float
        img_depth_float32 = np.asarray(img_depth_float, dtype=np.float32)
        img_rgb_string = img_depth_float32.tobytes()
    elif (cameraType == "DepthPlanner"):
        img_depth_float = data.image_data_float
        img_depth_float32 = np.asarray(img_depth_float, dtype=np.float32)
        img_rgb_string = img_depth_float32.tobytes()
    elif (cameraType == "DepthVis"):
        img_depth_float = data.image_data_float
        img_depth_float32 = np.asarray(img_depth_float, dtype=np.float32)
        img_rgb_string = img_depth_float32.tobytes()
    elif (cameraType == "Infrared"):
        img_rgb_string = data.image_data_uint8
    elif (cameraType == "SurfaceNormals"):
        img_rgb_string = data.image_data_uint8
    elif (cameraType == "DisparityNormalized"):
        img_depth_float = data.image_data_float
        img_depth_float32 = np.asarray(img_depth_float, dtype=np.float32)
        img_rgb_string = img_depth_float32.tobytes()
    else:
        img_rgb_string = data.image_data_uint8
    return img_rgb_string


def airsim_pub(rosRate, rosIMURate, activeTuple, topicsTuple, framesTuple, cameraSettingsTuple,
                lidarName, echoName, imuName, vehicleName, transformPose, carcontrolInputTopic, publishAlternative):

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
    encodingQuality = cameraSettingsTuple[12]

    client = airsimpy.CarClient()
    client.confirmConnection(rospy.get_name())

    rate = rospy.Rate(rosIMURate)

    periodFrames = round(float(rosIMURate) / float(rosRate))

    if camera1Active:
        sceneCamera1Pub = rospy.Publisher(sceneCamera1TopicName, Image, queue_size=1)
        if camera1Mono:
            sceneCamera1MonoPub = rospy.Publisher(sceneCamera1MonoTopicName, Image, queue_size=1)
        if not camera1SceneOnly:
            if not cameraSegmentationDisabled:
                rospy.logwarn("Camera1 with segmentation is enabled! Do not forget to generate instance segmentation data at the end if it is required!")
                segmentationCamera1Pub = rospy.Publisher(segmentationCamera1TopicName, Image, queue_size=1)
            depthCamera1Pub = rospy.Publisher(depthCamera1TopicName, Image, queue_size=1)
    if camera2Active:
        sceneCamera2Pub = rospy.Publisher(sceneCamera2TopicName, Image, queue_size=1)
        if camera2Mono:
            sceneCamera2MonoPub = rospy.Publisher(sceneCamera2MonoTopicName, Image, queue_size=1)
        if not camera2SceneOnly:
            if not cameraSegmentationDisabled:
                rospy.logwarn("Camera2 with segmentation is enabled! Do not forget to generate instance segmentation data at the end if it is required!")
                segmentationCamera2Pub = rospy.Publisher(segmentationCamera2TopicName, Image, queue_size=1)
            depthCamera2Pub = rospy.Publisher(depthCamera2TopicName, Image, queue_size=1)
    if camera3Active:
        sceneCamera3Pub = rospy.Publisher(sceneCamera3TopicName, Image, queue_size=1)
        if camera3Mono:
            sceneCamera3MonoPub = rospy.Publisher(sceneCamera3MonoTopicName, Image, queue_size=1)
        if not camera3SceneOnly:
            if not cameraSegmentationDisabled:
                rospy.logwarn("Camera3 with segmentation is enabled! Do not forget to generate instance segmentation data at the end if it is required!")
                segmentationCamera3Pub = rospy.Publisher(segmentationCamera3TopicName, Image, queue_size=1)
            depthCamera3Pub = rospy.Publisher(depthCamera3TopicName, Image, queue_size=1)
    if lidarActive or gpulidarActive:
        if gpulidarActive:
            rospy.logwarn("GPULidar is enabled! Do not forget to generate instance segmentation data at the end if it is required!")
        lidarPub = rospy.Publisher(lidarTopicName, PointCloud2, queue_size=1)
        lidarGroundtruthPub = rospy.Publisher(lidarGroundtruthTopicName, StringArray, queue_size=1)
    if echoActive:
        echoPub = rospy.Publisher(echoTopicName, PointCloud2, queue_size=1)
    if imuActive:
        imuPub = rospy.Publisher(imuTopicName, Imu, queue_size=1)
        if publishAlternative:
            imuAltPub = rospy.Publisher(imuAltTopicName, Imu, queue_size=1)
    if poseActive:
        if transformPose:
            rospy.Subscriber(poseTopicName, PoseStamped, handle_airsim_pose, (poseFrame, imuFrame))
        posePub = rospy.Publisher(poseTopicName, PoseStamped, queue_size=1)
        if publishAlternative:
            poseAltPub = rospy.Publisher(poseAltTopicName, PoseStamped, queue_size=1)
    if carcontrolActive:
        client.enableApiControl(True)
        queue_input_command = Queue(1)
        rospy.Subscriber(carcontrolInputTopic, Twist, handle_input_command, (queue_input_command))

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
        if not camera1SceneOnly:
            if not cameraSegmentationDisabled:
                responseLocations[1] = tmp_i; tmp_i += 1
                requests.append(airsimpy.ImageRequest(camera1Name, get_camera_type("Segmentation"), is_pixels_as_float("Segmentation"), False))
            responseLocations[2] = tmp_i; tmp_i += 1
            requests.append(airsimpy.ImageRequest(camera1Name, get_camera_type("DepthPlanner"), is_pixels_as_float("DepthPlanner"), False))
    if camera2Active:
        responseLocations[3] = tmp_i; tmp_i += 1
        requests.append(airsimpy.ImageRequest(camera2Name, get_camera_type("Scene"), is_pixels_as_float("Scene"), False))
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

    currentFrame = 1
    while not rospy.is_shutdown():

        timeStamp = rospy.Time.now()

        if currentFrame == periodFrames:
            currentFrame = 1

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

                img_rgb_string1_matrix = np.fromstring(img_rgb_string1, dtype=np.uint8).reshape(cameraHeight,
                                                                                                cameraWidth, 3)
                if encodingQuality > 0:
                    img_rgb_string1_matrix = cv2.cvtColor(img_rgb_string1_matrix, cv2.COLOR_RGB2BGR)
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), encodingQuality]
                    result, encimg = cv2.imencode('.jpg', img_rgb_string1_matrix, encode_param)
                    img_rgb_string1_matrix = cv2.imdecode(encimg, 1)
                    img_rgb_string1_matrix = cv2.cvtColor(img_rgb_string1_matrix, cv2.COLOR_BGR2RGB)

                if (len(img_rgb_string1) > 1):
                    msg = bridge.cv2_to_imgmsg(img_rgb_string1_matrix, encoding="rgb8")
                    msg.header.stamp = cameraTimeStamp
                    msg.header.frame_id = camera1Frame
                    sceneCamera1Pub.publish(msg)
                    if camera1Mono:
                        msg = bridge.cv2_to_imgmsg(cv2.cvtColor(img_rgb_string1_matrix, cv2.COLOR_RGB2GRAY),
                                                   encoding="mono8")
                        msg.header.stamp = cameraTimeStamp
                        msg.header.frame_id = camera1Frame
                        sceneCamera1MonoPub.publish(msg)
                    if not camera1SceneOnly:
                        if not cameraSegmentationDisabled:
                            msg.step = cameraWidth * 3
                            msg.data = img_rgb_string2
                            segmentationCamera1Pub.publish(msg)
                        msg.encoding = "32FC1"
                        msg.step = cameraWidth * 4
                        msg.data = img_rgb_string3
                        depthCamera1Pub.publish(msg)

            if camera2Active:
                if not camera2SceneOnly:
                    img_rgb_string1 = get_image_bytes(responses[responseLocations[3]], "Scene")
                    if not cameraSegmentationDisabled:
                        img_rgb_string2 = get_image_bytes(responses[responseLocations[4]], "Segmentation")
                    img_rgb_string3 = get_image_bytes(responses[responseLocations[5]], "DepthPlanner")
                else:
                    img_rgb_string1 = get_image_bytes(responses[responseLocations[3]], "Scene")

                img_rgb_string1_matrix = np.fromstring(img_rgb_string1, dtype=np.uint8).reshape(cameraHeight,
                                                                                                cameraWidth, 3)
                if encodingQuality > 0:
                    img_rgb_string1_matrix = cv2.cvtColor(img_rgb_string1_matrix, cv2.COLOR_RGB2BGR)
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), encodingQuality]
                    result, encimg = cv2.imencode('.jpg', img_rgb_string1_matrix, encode_param)
                    img_rgb_string1_matrix = cv2.imdecode(encimg, 1)
                    img_rgb_string1_matrix = cv2.cvtColor(img_rgb_string1_matrix, cv2.COLOR_BGR2RGB)

                if (len(img_rgb_string1) > 1):
                    msg = bridge.cv2_to_imgmsg(img_rgb_string1_matrix, encoding="rgb8")
                    msg.header.stamp = cameraTimeStamp
                    msg.header.frame_id = camera2Frame
                    sceneCamera2Pub.publish(msg)
                    if camera2Mono:
                        msg = bridge.cv2_to_imgmsg(cv2.cvtColor(img_rgb_string1_matrix, cv2.COLOR_RGB2GRAY),
                                                   encoding="mono8")
                        msg.header.stamp = cameraTimeStamp
                        msg.header.frame_id = camera2Frame
                        sceneCamera2MonoPub.publish(msg)
                    if not camera2SceneOnly:
                        if not cameraSegmentationDisabled:
                            msg.step = cameraWidth * 3
                            msg.data = img_rgb_string2
                            segmentationCamera2Pub.publish(msg)
                        msg.encoding = "32FC1"
                        msg.step = cameraWidth * 4
                        msg.data = img_rgb_string3
                        depthCamera2Pub.publish(msg)

            if camera3Active:
                if not camera3SceneOnly:
                    img_rgb_string1 = get_image_bytes(responses[responseLocations[6]], "Scene")
                    if not cameraSegmentationDisabled:
                        img_rgb_string2 = get_image_bytes(responses[responseLocations[7]], "Segmentation")
                    img_rgb_string3 = get_image_bytes(responses[responseLocations[8]], "DepthPlanner")
                else:
                    img_rgb_string1 = get_image_bytes(responses[responseLocations[6]], "Scene")

                img_rgb_string1_matrix = np.fromstring(img_rgb_string1, dtype=np.uint8).reshape(cameraHeight,
                                                                                                cameraWidth, 3)
                if encodingQuality > 0:
                    img_rgb_string1_matrix = cv2.cvtColor(img_rgb_string1_matrix, cv2.COLOR_RGB2BGR)
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), encodingQuality]
                    result, encimg = cv2.imencode('.jpg', img_rgb_string1_matrix, encode_param)
                    img_rgb_string1_matrix = cv2.imdecode(encimg, 1)
                    img_rgb_string1_matrix = cv2.cvtColor(img_rgb_string1_matrix, cv2.COLOR_BGR2RGB)

                if (len(img_rgb_string1) > 1):
                    msg = bridge.cv2_to_imgmsg(cv2.cvtColor(img_rgb_string1_matrix, cv2.COLOR_RGB2GRAY),
                                               encoding="mono8")
                    msg.header.stamp = cameraTimeStamp
                    msg.header.frame_id = camera3Frame
                    sceneCamera3Pub.publish(msg)
                    if camera3Mono:
                        msg = bridge.cv2_to_imgmsg(cv2.cvtColor(img_rgb_string1_matrix, cv2.COLOR_RGB2GRAY),
                                                   encoding="mono8")
                        msg.header.stamp = cameraTimeStamp
                        msg.header.frame_id = camera3Frame
                        sceneCamera3MonoPub.publish(msg)
                    if not camera3SceneOnly:
                        if not cameraSegmentationDisabled:
                            msg.step = cameraWidth * 3
                            msg.data = img_rgb_string2
                            segmentationCamera3Pub.publish(msg)
                        msg.encoding = "32FC1"
                        msg.step = cameraWidth * 4
                        msg.data = img_rgb_string3
                        depthCamera3Pub.publish(msg)

            if lidarActive or gpulidarActive:
                # get lidar data
                if lidarActive:
                    lidarData = client.getLidarData(lidarName, vehicleName)
                if gpulidarActive:
                    lidarData = client.getGPULidarData(lidarName, vehicleName)

                if lidarData.time_stamp != lastLidarTimeStamp:
                    # Check if there are any points in the data
                    if (len(lidarData.point_cloud) < 4):
                        lastLidarTimeStamp = lidarData.time_stamp
                    else:
                        lastLidarTimeStamp = lidarData.time_stamp

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
                        lidarPub.publish(pcloud)
                        lidarGroundtruthPub.publish(groundtruth)

            if echoActive:
                # get echo data
                echoData = client.getEchoData(echoName, vehicleName)

                if echoData.time_stamp != lastEchoTimeStamp:
                    # Check if there are any points in the data
                    if (len(echoData.point_cloud) < 4):
                        lastEchoTimeStamp = echoData.time_stamp
                    else:
                        lastEchoTimeStamp = echoData.time_stamp

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
                        echoPub.publish(pcloud)

        else:
            currentFrame += 1

        if imuActive:
            # get data of IMU sensor
            imuData = client.getImuData(imuName, vehicleName)

            # populate Imu ros message
            imu_msg = Imu()
            imu_msg.header.stamp = timeStamp
            imu_msg.header.frame_id = imuFrame
            imu_msg.orientation.x = imuData.orientation.inverse().x_val
            imu_msg.orientation.y = imuData.orientation.inverse().y_val
            imu_msg.orientation.z = imuData.orientation.inverse().z_val
            imu_msg.orientation.w = imuData.orientation.inverse().w_val

            imu_msg.angular_velocity.x = imuData.angular_velocity.x_val
            imu_msg.angular_velocity.y = -imuData.angular_velocity.y_val
            imu_msg.angular_velocity.z = -imuData.angular_velocity.z_val

            imu_msg.linear_acceleration.x = imuData.linear_acceleration.x_val
            imu_msg.linear_acceleration.y = -imuData.linear_acceleration.y_val
            imu_msg.linear_acceleration.z = -imuData.linear_acceleration.z_val

            # publish Imu message
            imuPub.publish(imu_msg)

            # publish Imu message in different coordinate scheme
            if publishAlternative:
                imu_msg.angular_velocity.x = -imuData.angular_velocity.z_val
                imu_msg.angular_velocity.y = imuData.angular_velocity.y_val
                imu_msg.angular_velocity.z = imuData.angular_velocity.x_val

                imu_msg.linear_acceleration.x = -imuData.linear_acceleration.z_val
                imu_msg.linear_acceleration.y = imuData.linear_acceleration.y_val
                imu_msg.linear_acceleration.z = imuData.linear_acceleration.x_val

                imuAltPub.publish(imu_msg)

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
            simPose.header.stamp = timeStamp
            simPose.header.seq = 1
            simPose.header.frame_id = poseFrame

            # publish PoseStamped message
            posePub.publish(simPose)

            # publish PoseStamped message in different coordinate scheme
            if publishAlternative:
                simPose.pose.position.x = -pos.z_val
                simPose.pose.position.y = pos.y_val
                simPose.pose.position.z = pos.x_val
                poseAltPub.publish(simPose)

        if carcontrolActive:
            # set the controls for car
            try:
                msg = queue_input_command.get_nowait()
                car_controls = airsimpy.CarControls()
                if (msg.linear.x < 0):
                    car_controls.is_manual_gear = True
                    car_controls.manual_gear = -1
                else:
                    car_controls.is_manual_gear = False
                car_controls.throttle = msg.linear.x
                car_controls.steering = -msg.angular.z
                client.setCarControls(car_controls, vehicleName)
            except:
                pass #queue is empty
        # sleep until next cycle
        rate.sleep()


if __name__ == '__main__':
    try:
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
        poseActive = rospy.get_param('~pose_active', 0)
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
        lidarGroundtruthTopicName = rospy.get_param('~topic_lidar_groundtruth', 'airsim/lidargroundtruth')
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
        encodingQuality = rospy.get_param('~camera_encoding_quality', 0)
        cameraSettingsTuple = (camera1Name, camera2Name, camera3Name, camera1SceneOnly, camera2SceneOnly, camera3SceneOnly,
                               camera1Mono, camera2Mono, camera3Mono, cameraHeight, cameraWidth,
                               cameraSegementationDisabled, encodingQuality)

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

        airsim_pub(rosRate, rosIMURate, activeTuple, topicsTuple, framesTuple, cameraSettingsTuple, lidarName, echoName, imuName,
                   vehicleName, transformPose, carcontrolInputTopic, publishAlternative)

    except rospy.ROSInterruptException:
        pass
