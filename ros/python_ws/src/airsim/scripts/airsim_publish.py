#!/usr/bin/env python

import setup_path
import airsimpy
import rospy
import time
import math
import threading
from std_msgs.msg import String, Header, Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped, TransformStamped, Point, Twist
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from airsim.msg import StringArray
from sensor_msgs.msg import PointCloud2, PointField, Imu, CameraInfo
from nav_msgs.msg import Odometry
from uwb_msgs.msg import Diagnostics, Range, RangeArray
from wifi_msgs.msg import DiagnosticsRSSI, RangeRSSI, RangeArrayRSSI
import rosbag
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import msgpackrpc
import sys
import tf
from tf.transformations import quaternion_from_euler
from multiprocessing import Queue



class PID(object):
    desired_speed = 0

    def __init__(self, kp, ki, kd, time, topicname):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.pe = 0
        self.ie = 0
        self.de = 0
        self.time = time
        self.integral = 0
        self.last_err = 0
        self.maxI = 1000000
        self.pid_queue = Queue(1)
        self.update_sub = rospy.Subscriber(topicname, Float32MultiArray, handle_input_command, (self.pid_queue))

    def update_params(self):

        # print("yeet")
        try:
            speed_msg = self.pid_queue.get_nowait()
        except:
            return
        #print(speed_msg)
        self.kp = speed_msg.data[0]
        self.ki = speed_msg.data[1]
        self.kd = speed_msg.data[2]

    def getPError(self, error):
        PError = self.kp * error
        return PError

    def getIError(self, error):
        self.integral += error
        if (math.fabs(self.integral) > self.maxI):
            if (self.integral > 0):
                self.integral = self.maxI
            else:
                self.integral = -self.maxI

        IError = self.ki * self.integral * self.time
        return IError

    def getDError(self, error):
        der = error - self.last_err
        self.last_err = error
        DError = self.kd * der / self.time
        return DError

    def getError(self, error):
        return self.getPError(error) + self.getIError(error) + self.getDError(error)


def Pose_2_mat(p):
    q = p.orientation
    pos = p.position
    T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
    T[:3, 3] = np.array([pos.x, pos.y, pos.z])
    return T


def T_inv(T_in):
    R_in = T_in[:3, :3]
    t_in = T_in[:3, [-1]]
    R_out = R_in.T
    t_out = -np.matmul(R_out, t_in)
    return np.vstack((np.hstack((R_out, t_out)), np.array([0, 0, 0, 1])))


def mat_2_tf(Mat, time, frame, child_frame):
    t = TransformStamped()
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(Mat)
    quat = tf.transformations.quaternion_from_euler(angles[0], angles[1], angles[2])
    t.header.stamp = time
    t.header.frame_id = frame
    t.child_frame_id = child_frame
    t.transform.translation.x = trans[0]
    t.transform.translation.y = trans[1]
    t.transform.translation.z = trans[2]
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    return t


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


def handle_input_command(msg, args):
    # set the controls for car
    q = args
    try:
        q.put(msg)
    except:
        print("Input queue full")


def airsim_publish(client, vehicle_name, pose_topic, pose_frame, tf_localisation_enable, carcontrol_enable, carcontrol_topic,
                   odometry_enable, odometry_topic,
                   sensor_imu_enable, sensor_imu_name, sensor_imu_topic,
                   sensor_imu_frame, sensor_echo_names,
                   sensor_echo_topics, sensor_echo_frames, sensor_lidar_names,
                   sensor_lidar_toggle_segmentation,
                   sensor_lidar_topics, sensor_lidar_segmentation_topics, sensor_lidar_frames,
                   sensor_gpulidar_names, sensor_gpulidar_topics,
                   sensor_gpulidar_frames, sensor_camera_names, sensor_camera_toggle_scene_mono,
                   sensor_camera_scene_quality, sensor_camera_toggle_segmentation,
                   sensor_camera_toggle_depth, sensor_camera_scene_topics,
                   sensor_camera_segmentation_topics, sensor_camera_depth_topics,
                   sensor_camera_frames, sensor_camera_optical_frames, sensor_camera_toggle_camera_info, sensor_camera_info_topics, sensor_stereo_enable, baseline,
                   object_names, objects_coordinates_local, object_topics):
    rate = rospy.Rate(ros_rate)

    speed_pid = PID(1, 0, 0.001, 1.0 / ros_rate, "/speed_pid")

    # steering_pid = PID(0.0, 0.3, 0.05, 1.0/ros_rate, "/steer_pid")


    last_timestamps = {}
    warning_issued = {}
    pose_publisher = rospy.Publisher(pose_topic, PoseStamped, queue_size=1)
    if sensor_imu_enable:
        imu_publisher = rospy.Publisher(sensor_imu_topic, Imu, queue_size=1)
    if odometry_enable:
        odomPub = rospy.Publisher(odometry_topic, Odometry, queue_size=1)
    if tf_localisation_enable:
        tf_br = tf2_ros.TransformBroadcaster()
    objectpose_publishers = {}
    pointcloud_publishers = {}
    string_segmentation_publishers = {}
    image_publishers = {}
    cameraInfo_objects = {}
    for sensor_index, sensor_name in enumerate(sensor_echo_names):
        pointcloud_publishers[sensor_name] = rospy.Publisher(sensor_echo_topics[sensor_index], PointCloud2,
                                                             queue_size=2)
        last_timestamps[sensor_name] = None
    for sensor_index, sensor_name in enumerate(sensor_lidar_names):
        pointcloud_publishers[sensor_name] = rospy.Publisher(sensor_lidar_topics[sensor_index], PointCloud2,
                                                             queue_size=2)
        last_timestamps[sensor_name] = None
        if sensor_lidar_toggle_segmentation[sensor_index] is 1:
            string_segmentation_publishers[sensor_name] = \
                rospy.Publisher(sensor_lidar_segmentation_topics[sensor_index], StringArray, queue_size=1)
    for sensor_index, sensor_name in enumerate(sensor_gpulidar_names):
        pointcloud_publishers[sensor_name] = rospy.Publisher(sensor_gpulidar_topics[sensor_index], PointCloud2,
                                                             queue_size=2)
        last_timestamps[sensor_name] = None

    if (len(sensor_uwb_names) > 0):
        uwb_rangeArray_publisher = rospy.Publisher(sensor_uwb_topic, RangeArray, queue_size=10)
        
    if (len(sensor_uwb_names) > 0):
        wifi_rangeArray_publisher = rospy.Publisher(sensor_wifi_topic, RangeArray, queue_size=10)

    for sensor_index, sensor_name in enumerate(sensor_camera_names):
        image_publishers[sensor_name + '_scene'] = rospy.Publisher(sensor_camera_scene_topics[sensor_index],
                                                                   Image, queue_size=2)
        if sensor_camera_toggle_segmentation[sensor_index] is 1:
            image_publishers[sensor_name + '_segmentation'] = rospy.Publisher(sensor_camera_segmentation_topics[sensor_index],
                                                                              Image, queue_size=2)
        if sensor_camera_toggle_depth[sensor_index] is 1:
            image_publishers[sensor_name + '_depth'] = rospy.Publisher(sensor_camera_depth_topics[sensor_index],
                                                                       Image, queue_size=2)
        if sensor_camera_toggle_camera_info[sensor_index] is 1:
            image_publishers[sensor_name + '_cameraInfo'] = rospy.Publisher(sensor_camera_info_topics[sensor_index], CameraInfo, queue_size=2)
            cameraInfo_objects[sensor_name] = client.simGetCameraInfo(sensor_name)

    for object_index, object_name in enumerate(object_names):
        objectpose_publishers[object_name] = rospy.Publisher(object_topics[object_index], PoseStamped, queue_size=1)
        warning_issued[object_name] = False

    cv_bridge = CvBridge()

    fields_echo = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('a', 12, PointField.FLOAT32, 1),
        PointField('d', 16, PointField.FLOAT32, 1)
    ]

    fields_lidar = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('rgb', 12, PointField.UINT32, 1),
        PointField('intensity', 16, PointField.FLOAT32, 1)
    ]

    requests = []
    response_locations = {}
    response_index = 0
    for sensor_index, sensor_name in enumerate(sensor_camera_names):
        response_locations[sensor_name + '_scene'] = response_index
        response_index += 1
        requests.append(airsimpy.ImageRequest(sensor_name, get_camera_type("Scene"),
                                              is_pixels_as_float("Scene"), False))
        if sensor_camera_toggle_segmentation[sensor_index] is 1:
            requests.append(airsimpy.ImageRequest(sensor_name, get_camera_type("Segmentation"),
                                                  is_pixels_as_float("Segmentation"), False))
            response_locations[sensor_name + '_segmentation'] = response_index
            response_index += 1
        if sensor_camera_toggle_depth[sensor_index] is 1:
            requests.append(airsimpy.ImageRequest(sensor_name, get_camera_type("DepthPlanner"),
                                                  is_pixels_as_float("DepthPlanner"), False))
            response_locations[sensor_name + '_depth'] = response_index
            response_index += 1
    if carcontrol_enable == 1:
        print("Control of car enabled")
        desired_rotation = 0
        client.enableApiControl(True)
        queue_input_command = Queue(1)
        rospy.Subscriber(carcontrol_topic, Twist, handle_input_command, (queue_input_command))
        requestpub = rospy.Publisher("/req_speed", Float32, queue_size=1)
        realpub = rospy.Publisher("/real_speed", Float32, queue_size=1)
        errorpub = rospy.Publisher("/speed_error", Float32, queue_size=1)

    # rospy.loginfo("Started publishers...")

    while not rospy.is_shutdown():

        timestamp = rospy.Time.now()

        try:
            pose = client.simGetVehiclePose(vehicle_name)
        except msgpackrpc.error.RPCError:
            rospy.logerr("vehicle '" + vehicle_name + "' could not be found.")
            rospy.signal_shutdown('Vehicle not found.')
            sys.exit()

        pos = pose.position
        orientation = pose.orientation.inverse()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = pose_frame
        pose_msg.header.seq = 1

        pose_msg.pose.position.x = pos.x_val
        pose_msg.pose.position.y = -pos.y_val
        pose_msg.pose.position.z = -pos.z_val

        pose_msg.pose.orientation.w = orientation.w_val
        pose_msg.pose.orientation.x = orientation.x_val
        pose_msg.pose.orientation.y = orientation.y_val
        pose_msg.pose.orientation.z = orientation.z_val

        pose_publisher.publish(pose_msg)
        T_w_veh = Pose_2_mat(pose_msg.pose)

        simOdom = Odometry()

        if odometry_enable:
            simOdom.header = pose_msg.header
            simOdom.child_frame_id = vehicle_base_frame
            simOdom.pose.pose = pose_msg.pose
            kinematics = client.simGetGroundTruthKinematics(vehicle_name)
            T_veh_w = T_inv(T_w_veh)

            local_linear = np.matmul(T_veh_w[:3, :3], np.array([kinematics.linear_velocity.x_val, - kinematics.linear_velocity.y_val, - kinematics.linear_velocity.z_val]))
            local_angular = np.matmul(T_veh_w[:3, :3], np.array([kinematics.angular_velocity.x_val, - kinematics.angular_velocity.y_val, - kinematics.angular_velocity.z_val]))
            simOdom.twist.twist.linear.x = local_linear[0]
            simOdom.twist.twist.linear.y = local_linear[1]
            simOdom.twist.twist.linear.z = local_linear[2]
            simOdom.twist.twist.angular.x = local_angular[0]
            simOdom.twist.twist.angular.y = local_angular[1]
            simOdom.twist.twist.angular.z = local_angular[2]
            odomPub.publish(simOdom)

        if tf_localisation_enable:
            tf_odom_pose = mat_2_tf(T_w_veh, timestamp, "odom", vehicle_base_frame)
            tf_br.sendTransform(tf_odom_pose)

            # tf_map_odom = TransformStamped()
            # tf_map_odom.header.stamp = timestamp
            # tf_map_odom.header.frame_id = pose_frame
            # tf_map_odom.child_frame_id = "odom"
            # tf_map_odom.transform.rotation.w = 1
            # tf_br.sendTransform(tf_map_odom)


        if sensor_imu_enable:

            try:
                imu_data = client.getImuData(sensor_imu_name, vehicle_name)
            except msgpackrpc.error.RPCError:
                rospy.logerr("IMU sensor '" + sensor_imu_name + "' could not be found.")
                rospy.signal_shutdown('Sensor not found.')
                sys.exit()

            imu_msg = Imu()
            imu_msg.header.stamp = timestamp
            imu_msg.header.frame_id = sensor_imu_frame

            imu_msg.orientation.x = imu_data.orientation.inverse().x_val
            imu_msg.orientation.y = imu_data.orientation.inverse().y_val
            imu_msg.orientation.z = imu_data.orientation.inverse().z_val
            imu_msg.orientation.w = imu_data.orientation.inverse().w_val

            imu_msg.angular_velocity.x = imu_data.angular_velocity.x_val
            imu_msg.angular_velocity.y = -imu_data.angular_velocity.y_val
            imu_msg.angular_velocity.z = -imu_data.angular_velocity.z_val

            imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x_val
            imu_msg.linear_acceleration.y = -imu_data.linear_acceleration.y_val
            imu_msg.linear_acceleration.z = -imu_data.linear_acceleration.z_val

            imu_publisher.publish(imu_msg)

        camera_responses = client.simGetImages(requests)

        for sensor_index, sensor_name in enumerate(sensor_camera_names):
            response = camera_responses[response_locations[sensor_name + '_scene']]
            if response.width == 0 and response.height == 0:
                rospy.logwarn("Camera '" + sensor_name + "' could not retrieve scene image.")
            else:
                rgb_matrix = np.frombuffer(get_image_bytes(response, "Scene"), dtype=np.uint8).reshape(response.height,
                                                                                                       response.width, 3)
                if sensor_camera_scene_quality[sensor_index] > 0:
                    rgb_matrix = cv2.cvtColor(rgb_matrix, cv2.COLOR_RGB2BGR)
                    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), sensor_camera_scene_quality[sensor_index]]
                    result, img = cv2.imencode('.jpg', rgb_matrix, encode_params)
                    rgb_matrix = cv2.imdecode(img, 1)
                    rgb_matrix = cv2.cvtColor(rgb_matrix, cv2.COLOR_BGR2RGB)

                if sensor_camera_toggle_scene_mono is 1:
                    camera_msg = cv_bridge.cv2_to_imgmsg(cv2.cvtColor(rgb_matrix, cv2.COLOR_RGB2GRAY), encoding="mono8")
                else:
                    camera_msg = cv_bridge.cv2_to_imgmsg(rgb_matrix, encoding="rgb8")
                camera_msg.header.stamp = timestamp
                camera_msg.header.frame_id = sensor_camera_optical_frames[sensor_index]
                image_publishers[sensor_name + '_scene'].publish(camera_msg)

            if sensor_camera_toggle_segmentation[sensor_index] is 1:
                response = camera_responses[response_locations[sensor_name + '_segmentation']]
                if response.width == 0 and response.height == 0:
                    rospy.logwarn("Camera '" + sensor_name + "' could not retrieve segmentation image.")
                else:
                    rgb_string = get_image_bytes(response, "Segmentation")
                    camera_msg.step = response.width * 3
                    camera_msg.data = rgb_string
                    image_publishers[sensor_name + '_segmentation'].publish(camera_msg)
            if sensor_camera_toggle_depth[sensor_index] is 1:
                response = camera_responses[response_locations[sensor_name + '_depth']]
                if response.width == 0 and response.height == 0:
                    rospy.logwarn("Camera '" + sensor_name + "' could not retrieve depth image.")
                else:
                    rgb_string = get_image_bytes(response, "DepthPlanner")
                    camera_msg.encoding = "32FC1"
                    camera_msg.step = response.width * 4
                    camera_msg.data = rgb_string
                    image_publishers[sensor_name + '_depth'].publish(camera_msg)
            if sensor_camera_toggle_camera_info[sensor_index] is 1:
                FOV = cameraInfo_objects[sensor_name].fov
                cam_info_msg = CameraInfo()
                cam_info_msg.header.frame_id = sensor_camera_optical_frames[sensor_index]
                cam_info_msg.header.stamp = timestamp
                cam_info_msg.height = response.height
                cam_info_msg.width = response.width
                f = (cam_info_msg.width / 2.0) / math.tan(FOV * math.pi / 360)
                if sensor_stereo_enable and sensor_index == 1:
                    Tx = -f * baseline
                else:
                    Tx = 0
                cam_info_msg.K = [f, 0.0, cam_info_msg.width / 2.0,
                                  0.0, f, cam_info_msg.height / 2.0,
                                  0.0, 0.0, 1.0]
                cam_info_msg.P = [f, 0.0, cam_info_msg.width / 2.0, Tx,
                                  0.0, f, cam_info_msg.height / 2.0, 0.0,
                                  0.0, 0.0, 1.0, 0.0]
                cam_info_msg.D = [0, 0, 0, 0, 0]  # in future get from client.simGetDistortionParams(camera1Name)
                cam_info_msg.distortion_model = 'plumb_bob'
                cam_info_msg.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
                image_publishers[sensor_name + '_cameraInfo'].publish(cam_info_msg)

        for sensor_index, sensor_name in enumerate(sensor_echo_names):
            echo_data = client.getEchoData(sensor_name, vehicle_name)

            if echo_data.time_stamp != last_timestamps[sensor_name]:
                if len(echo_data.point_cloud) < 4:
                    last_timestamps[sensor_name] = timestamp
                else:
                    last_timestamps[sensor_name] = timestamp

                    points = np.array(echo_data.point_cloud, dtype=np.dtype('f4'))
                    points = np.reshape(points, (int(points.shape[0] / 5), 5))
                    points = points * np.array([1, -1, -1, 1, 1])
                    points_list = points.tolist()
                    header = Header()
                    header.frame_id = sensor_echo_frames[sensor_index]
                    pcloud = pc2.create_cloud(header, fields_echo, points_list)
                    pcloud.header.stamp = timestamp
                    pointcloud_publishers[sensor_name].publish(pcloud)

        for sensor_index, sensor_name in enumerate(sensor_lidar_names):
            lidar_data = client.getLidarData(sensor_name, vehicle_name)

            if lidar_data.time_stamp != last_timestamps[sensor_name]:
                if len(lidar_data.point_cloud) < 4:
                    last_timestamps[sensor_name] = lidar_data.time_stamp
                else:
                    last_timestamps[sensor_name] = lidar_data.time_stamp

                    pcloud = PointCloud2()
                    points = np.array(lidar_data.point_cloud, dtype=np.dtype('f4'))
                    points = np.reshape(points, (int(points.shape[0] / 3), 3))
                    points = points * np.array([1, -1, -1])
                    cloud = points.tolist()
                    pcloud.header.frame_id = sensor_lidar_frames[sensor_index]
                    pcloud.header.stamp = timestamp
                    pcloud = pc2.create_cloud_xyz32(pcloud.header, cloud)

                    pointcloud_publishers[sensor_name].publish(pcloud)

                    if sensor_lidar_toggle_segmentation[sensor_index] is 1:
                        labels = np.array(lidar_data.groundtruth, dtype=np.dtype('U'))
                        groundtruth = StringArray()
                        groundtruth.data = labels.tolist()
                        groundtruth.header.frame_id = sensor_lidar_frames[sensor_index]
                        groundtruth.header.stamp = timestamp
                        string_segmentation_publishers[sensor_name].publish(groundtruth)

        for sensor_index, sensor_name in enumerate(sensor_gpulidar_names):
            lidar_data = client.getGPULidarData(sensor_name, vehicle_name)

            if lidar_data.time_stamp != last_timestamps[sensor_name]:
                if len(lidar_data.point_cloud) < 5:
                    last_timestamps[sensor_name] = lidar_data.time_stamp
                else:
                    last_timestamps[sensor_name] = lidar_data.time_stamp

                    pcloud = PointCloud2()
                    points = np.array(lidar_data.point_cloud, dtype=np.dtype('f4'))
                    points = np.reshape(points, (int(points.shape[0] / 5), 5))
                    pcloud.header.frame_id = sensor_gpulidar_frames[sensor_index]
                    pcloud.header.stamp = timestamp
                    pcloud = pc2.create_cloud(pcloud.header, fields_lidar, points.tolist())

                    pointcloud_publishers[sensor_name].publish(pcloud)

        for sensor_index, sensor_name in enumerate(sensor_uwb_names):
            if (sensor_index == 0):  # only once
                uwb_data = client.getUWBData()

                # Sanity check
                if len(uwb_data.mur_anchorPosX) != len(uwb_data.mur_anchorPosY) or \
                        len(uwb_data.mur_anchorPosX) != len(uwb_data.mur_anchorPosZ) or \
                        len(uwb_data.mur_anchorPosX) != len(uwb_data.mur_distance) or \
                        len(uwb_data.mur_anchorPosX) != len(uwb_data.mur_rssi) or \
                        len(uwb_data.mur_anchorPosX) != len(uwb_data.mur_time_stamp) or \
                        len(uwb_data.mur_anchorPosX) != len(uwb_data.mur_anchorId) or \
                        len(uwb_data.mur_anchorPosX) != len(uwb_data.mur_valid_range):
                    rospy.logerr("UWB sensor mur lengths do not match")
                    rospy.signal_shutdown('Packet error.')
                    sys.exit()
                if len(uwb_data.mura_ranges) != len(uwb_data.mura_tagId) or \
                        len(uwb_data.mura_ranges) != len(uwb_data.mura_tagPosX) or \
                        len(uwb_data.mura_ranges) != len(uwb_data.mura_tagPosY) or \
                        len(uwb_data.mura_ranges) != len(uwb_data.mura_tagPosZ):
                    rospy.logerr("UWB sensor mura lengths do not match")
                    rospy.signal_shutdown('Packet error.')
                    sys.exit()

                uwb_data.mur_anchorId = np.array(uwb_data.mur_anchorId)

                mur_time_stamp = []
                mur_anchorId = []
                mur_anchorPosX = []
                mur_anchorPosY = []
                mur_anchorPosZ = []
                mur_valid_range = []
                mur_distance = []
                mur_rssi = []
                mura_ranges = []

                idx_offset = 0
                
                for rangeIdx, ranges in enumerate(uwb_data.mura_ranges):
                    u, uniqueRangeIds = np.unique(uwb_data.mur_anchorId[ranges], return_index=True)
                    uniqueRangeIds += idx_offset

                    mura_ranges_idx_start = len(mur_anchorPosX)
                    mur_anchorPosX  += list(np.array(uwb_data.mur_anchorPosX)[uniqueRangeIds])
                    mur_anchorPosY  += list(np.array(uwb_data.mur_anchorPosY)[uniqueRangeIds])
                    mur_anchorPosZ  += list(np.array(uwb_data.mur_anchorPosZ)[uniqueRangeIds])
                    mur_time_stamp  += list(np.array(uwb_data.mur_time_stamp)[uniqueRangeIds])
                    mur_anchorId    += list(np.array(uwb_data.mur_anchorId)[uniqueRangeIds])
                    mur_valid_range += list(np.array(uwb_data.mur_valid_range)[uniqueRangeIds])

                    mura_ranges.append([])
                    mura_ranges[rangeIdx] = list(range(mura_ranges_idx_start, len(mur_anchorPosX)))
                    for arange in uniqueRangeIds:
                        currentRanges = np.array(uwb_data.mur_anchorId)[ranges]
                        currentRssi = np.array(uwb_data.mur_rssi)[ranges]
                        currentDistance = np.array(uwb_data.mur_distance)[ranges]
                        currentRssiFiltered = currentRssi * list(map(int, currentRanges == uwb_data.mur_anchorId[arange]))
                        maxRssi = max(currentRssiFiltered)
                        maxRssiIdx = currentRssiFiltered.argmax()

                        mur_distance.append(currentDistance[maxRssiIdx])
                        mur_rssi.append(maxRssi)

                    # for arange in uniqueRangeIds:
                    
                    idx_offset += len(ranges)
                    
                for mura_idx in range(0, len(uwb_data.mura_tagId)):
                    rangeArray = RangeArray()
                    rangeArray.tagid = str(uwb_data.mura_tagId[mura_idx])
                    rangeArray.tag_position = Point(uwb_data.mura_tagPosX[mura_idx], uwb_data.mura_tagPosY[mura_idx], uwb_data.mura_tagPosZ[mura_idx])
                    rangeArray.header.stamp = timestamp
                    
                    #for mur_id in range(0, len(mur_time_stamp)):
                    for mur_id in mura_ranges[mura_idx]:
                        diag = Diagnostics()
                        diag.rssi = mur_rssi[mur_id]
                        
                        rang = Range()
                        #rang.stamp = mur_time_stamp[mur_id]
                        rang.stamp = timestamp
                        rang.anchorid = str(mur_anchorId[mur_id])
                        rang.anchor_position = Point(mur_anchorPosX[mur_id], mur_anchorPosY[mur_id], mur_anchorPosZ[mur_id])
                        rang.valid_range = mur_valid_range[mur_id]
                        rang.distance = mur_distance[mur_id]
                        rang.diagnostics = diag

                        rangeArray.ranges.append(rang) 

                    uwb_rangeArray_publisher.publish(rangeArray)

                #rospy.logerr("run once")
                #rospy.signal_shutdown('Packet error.')
                #sys.exit()
        for sensor_index, sensor_name in enumerate(sensor_wifi_names):
            if (sensor_index == 0):  # only once
                wifi_data = client.getWifiData()

                # Sanity check
                if len(wifi_data.wr_anchorPosX) != len(wifi_data.wr_anchorPosY) or \
                        len(wifi_data.wr_anchorPosX) != len(wifi_data.wr_anchorPosZ) or \
                        len(wifi_data.wr_anchorPosX) != len(wifi_data.wr_distance) or \
                        len(wifi_data.wr_anchorPosX) != len(wifi_data.wr_rssi) or \
                        len(wifi_data.wr_anchorPosX) != len(wifi_data.wr_time_stamp) or \
                        len(wifi_data.wr_anchorPosX) != len(wifi_data.wr_anchorId) or \
                        len(wifi_data.wr_anchorPosX) != len(wifi_data.wr_valid_range):
                    rospy.logerr("Wifi sensor wr lengths do not match")
                    rospy.signal_shutdown('Packet error.')
                    sys.exit()
                if len(wifi_data.wra_ranges) != len(wifi_data.wra_tagId) or \
                        len(wifi_data.wra_ranges) != len(wifi_data.wra_tagPosX) or \
                        len(wifi_data.wra_ranges) != len(wifi_data.wra_tagPosY) or \
                        len(wifi_data.wra_ranges) != len(wifi_data.wra_tagPosZ):
                    rospy.logerr("Wifi sensor wra lengths do not match")
                    rospy.signal_shutdown('Packet error.')
                    sys.exit()

                wifi_data.wr_anchorId = np.array(wifi_data.wr_anchorId)

                wr_time_stamp = []
                wr_anchorId = []
                wr_anchorPosX = []
                wr_anchorPosY = []
                wr_anchorPosZ = []
                wr_valid_range = []
                wr_distance = []
                wr_rssi = []
                wra_ranges = []

                idx_offset = 0
                
                for rangeIdx, ranges in enumerate(wifi_data.wra_ranges):
                    u, uniqueRangeIds = np.unique(wifi_data.wr_anchorId[ranges], return_index=True)
                    uniqueRangeIds += idx_offset

                    wra_ranges_idx_start = len(wr_anchorPosX)
                    wr_anchorPosX  += list(np.array(wifi_data.wr_anchorPosX)[uniqueRangeIds])
                    wr_anchorPosY  += list(np.array(wifi_data.wr_anchorPosY)[uniqueRangeIds])
                    wr_anchorPosZ  += list(np.array(wifi_data.wr_anchorPosZ)[uniqueRangeIds])
                    wr_time_stamp  += list(np.array(wifi_data.wr_time_stamp)[uniqueRangeIds])
                    wr_anchorId    += list(np.array(wifi_data.wr_anchorId)[uniqueRangeIds])
                    wr_valid_range += list(np.array(wifi_data.wr_valid_range)[uniqueRangeIds])

                    wra_ranges.append([])
                    wra_ranges[rangeIdx] = list(range(wra_ranges_idx_start, len(wr_anchorPosX)))
                    for arange in uniqueRangeIds:
                        currentRanges = np.array(wifi_data.wr_anchorId)[ranges]
                        currentRssi = np.array(wifi_data.wr_rssi)[ranges]
                        currentDistance = np.array(wifi_data.wr_distance)[ranges]
                        currentRssiFiltered = currentRssi * list(map(int, currentRanges == wifi_data.wr_anchorId[arange]))
                        maxRssi = max(currentRssiFiltered)
                        maxRssiIdx = currentRssiFiltered.argmax()

                        wr_distance.append(currentDistance[maxRssiIdx])
                        wr_rssi.append(maxRssi)

                    # for arange in uniqueRangeIds:
                    
                    idx_offset += len(ranges)
                    
                for wra_idx in range(0, len(wifi_data.wra_tagId)):
                    rangeArray = RangeArray()
                    rangeArray.tagid = str(wifi_data.wra_tagId[wra_idx])
                    rangeArray.tag_position = Point(wifi_data.wra_tagPosX[wra_idx], wifi_data.wra_tagPosY[wra_idx], wifi_data.wra_tagPosZ[wra_idx])
                    rangeArray.header.stamp = timestamp
                    
                    #for wr_id in range(0, len(wr_time_stamp)):
                    for wr_id in wra_ranges[wra_idx]:
                        diag = Diagnostics()
                        diag.rssi = wr_rssi[wr_id]
                        
                        rang = Range()
                        #rang.stamp = wr_time_stamp[wr_id]
                        rang.stamp = timestamp
                        rang.anchorid = str(wr_anchorId[wr_id])
                        rang.anchor_position = Point(wr_anchorPosX[wr_id], wr_anchorPosY[wr_id], wr_anchorPosZ[wr_id])
                        rang.valid_range = wr_valid_range[wr_id]
                        rang.distance = wr_distance[wr_id]
                        rang.diagnostics = diag

                        rangeArray.ranges.append(rang) 

                    wifi_rangeArray_publisher.publish(rangeArray)

                #rospy.logerr("run once")
                #rospy.signal_shutdown('Packet error.')
                #sys.exit()
            
        for object_index, object_name in enumerate(object_names):
            if objects_coordinates_local[object_index] == 1:
                pose = client.simGetObjectPose(object_name, True)
            else:
                pose = client.simGetObjectPose(object_name, False)
            if np.isnan(pose.position.x_val):
                if warning_issued[object_name] is False:
                    rospy.logwarn("Object '" + object_name + "' could not be found.")
                    warning_issued[object_name] = True
            else:
                warning_issued[object_name] = False
                pos = pose.position
                orientation = pose.orientation.inverse()

                object_pose = PoseStamped()
                object_pose.pose.position.x = pos.x_val
                object_pose.pose.position.y = -pos.y_val
                object_pose.pose.position.z = pos.z_val
                object_pose.pose.orientation.w = orientation.w_val
                object_pose.pose.orientation.x = orientation.x_val
                object_pose.pose.orientation.y = orientation.y_val
                object_pose.pose.orientation.z = orientation.z_val
                object_pose.header.stamp = timestamp
                object_pose.header.seq = 1
                object_pose.header.frame_id = pose_frame

                objectpose_publishers[object_name].publish(object_pose)

        if carcontrol_enable == 1:
            # set the controls for car
            car_controls = airsimpy.CarControls()
            car_controls.is_manual_gear = True
            car_controls.manual_gear = 1

            try:
                # if queue_input_command.():
                #    msg.linear.x = 0
                msg = queue_input_command.get_nowait()

                car_controls.is_manual_gear = True
                speed_pid.desired_speed = msg.linear.x
                desired_rotation = -msg.angular.z

            except:
                pass  # queue is empty

            speed_pid.update_params()
            v = simOdom.twist.twist.linear.x
            error = speed_pid.desired_speed - v
            desired_accel = speed_pid.getError(error)
            # if abs(desired_accel) <= 0.05:
            #     desired_accel = 0
            wheelbase = 0.3
            if desired_rotation == 0 or abs(v) <= 0.1:
                steer_angle = desired_rotation
            else:
                radius = v / desired_rotation
                steer_angle = math.atan(wheelbase / radius)
            # steer_angle = desired_rotation

            if abs(steer_angle) <= 0.05:
                steer_angle = 0
            car_controls.steering = steer_angle

            if speed_pid.desired_speed == 0 and abs(v)<=0.1 and desired_rotation == 0:
                car_controls.brake = 0.1
                car_controls.throttle = 0.0
            else:
                car_controls.throttle = desired_accel
                car_controls.brake = 0.0
            requestpub.publish(speed_pid.desired_speed)
            realpub.publish(v)
            errorpub.publish(error)

            client.setCarControls(car_controls, vehicle_name)

        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('airsim_play_route_record_sensors', anonymous=True)

        ros_rate = rospy.get_param('~rate', "True")

        ip = rospy.get_param('~ip', "True")

        toggle_drone = rospy.get_param('~toggle_drone', "True")

        vehicle_name = rospy.get_param('~vehicle_name', "True")
        vehicle_base_frame = rospy.get_param('~vehicle_base_frame', "True")

        pose_topic = rospy.get_param('~pose_topic', "True")
        pose_frame = rospy.get_param('~pose_frame', "True")
        tf_localisation_enable = rospy.get_param('~tf_localisation_enable', "True")

        carcontrol_enable = rospy.get_param('~carcontrol_enable', "True")
        carcontrol_topic = rospy.get_param('~carcontrol_topic', "True")

        odometry_enable = rospy.get_param('~odometry_enable', "True")
        odometry_topic = rospy.get_param('~odometry_topic', "True")

        sensor_imu_enable = rospy.get_param('~sensor_imu_enable', "True")
        sensor_imu_name = rospy.get_param('~sensor_imu_name', "True")
        sensor_imu_topic = rospy.get_param('~sensor_imu_topic', "True")
        sensor_imu_frame = rospy.get_param('~sensor_imu_frame', "True")

        sensor_echo_names = rospy.get_param('~sensor_echo_names', "True")
        sensor_echo_topics = rospy.get_param('~sensor_echo_topics', "True")
        sensor_echo_frames = rospy.get_param('~sensor_echo_frames', "True")

        sensor_lidar_names = rospy.get_param('~sensor_lidar_names', "True")
        sensor_lidar_toggle_groundtruth = rospy.get_param('~sensor_lidar_toggle_groundtruth', "True")
        sensor_lidar_topics = rospy.get_param('~sensor_lidar_topics', "True")
        sensor_lidar_segmentation_topics = rospy.get_param('~sensor_lidar_segmentation_topics', "True")
        sensor_lidar_frames = rospy.get_param('~sensor_lidar_frames', "True")

        sensor_gpulidar_names = rospy.get_param('~sensor_gpulidar_names', "True")
        sensor_gpulidar_topics = rospy.get_param('~sensor_gpulidar_topics', "True")
        sensor_gpulidar_frames = rospy.get_param('~sensor_gpulidar_frames', "True")

        sensor_uwb_names = rospy.get_param('~sensor_uwb_names', "True")
        sensor_uwb_topic = rospy.get_param('~sensor_uwb_topic', "True")
        sensor_uwb_frames = rospy.get_param('~sensor_uwb_frames', "True")

        sensor_wifi_names = rospy.get_param('~sensor_wifi_names', "True")
        sensor_wifi_topic = rospy.get_param('~sensor_wifi_topic', "True")
        sensor_wifi_frames = rospy.get_param('~sensor_wifi_frames', "True")

        sensor_camera_names = rospy.get_param('~sensor_camera_names', "True")
        sensor_camera_toggle_scene_mono = rospy.get_param('~sensor_camera_toggle_scene_mono', "True")
        sensor_camera_scene_quality = rospy.get_param('~sensor_camera_scene_quality', "True")
        sensor_camera_toggle_segmentation = rospy.get_param('~sensor_camera_toggle_segmentation', "True")
        sensor_camera_toggle_depth = rospy.get_param('~sensor_camera_toggle_depth', "True")
        sensor_camera_scene_topics = rospy.get_param('~sensor_camera_scene_topics', "True")
        sensor_camera_segmentation_topics = rospy.get_param('~sensor_camera_segmentation_topics', "True")
        sensor_camera_depth_topics = rospy.get_param('~sensor_camera_depth_topics', "True")
        sensor_camera_frames = rospy.get_param('~sensor_camera_frames', "True")
        sensor_camera_optical_frames = rospy.get_param('~sensor_camera_optical_frames', "True")
        sensor_camera_toggle_camera_info = rospy.get_param('~sensor_camera_toggle_camera_info', "True")
        sensor_camera_info_topics = rospy.get_param('~sensor_camera_info_topics', "True")
        sensor_stereo_enable = rospy.get_param('~sensor_stereo_enable', "True")

        object_names = rospy.get_param('~object_names', "True")
        objects_coordinates_local = rospy.get_param('~objects_coordinates_local', "True")
        object_topics = rospy.get_param('~object_topics', "True")

        rospy.loginfo("Connecting to AirSim...")
        if toggle_drone:
            client = airsimpy.MultirotorClient(ip, timeout_value=15)
        else:
            client = airsimpy.CarClient(ip, timeout_value=15)
        try:
            client.confirmConnection(rospy.get_name())
        except msgpackrpc.error.TimeoutError:
            rospy.logerr("Could not connect to AirSim.")
            rospy.signal_shutdown('no connection to airsim.')
            sys.exit()
        rospy.loginfo("Connected to AirSim!")

        rospy.loginfo("Starting static transforms...")
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        transform_list = []

        for sensor_index, sensor_name in enumerate(sensor_echo_names):
            try:
                echo_data = client.getEchoData(sensor_name, vehicle_name)
            except msgpackrpc.error.RPCError:
                rospy.logerr("Echo sensor '" + sensor_name + "' could not be found.")
                rospy.signal_shutdown('Sensor not found.')
                sys.exit()
            pose = echo_data.pose
            orientation = pose.orientation.inverse()
            static_transform = TransformStamped()
            static_transform.header.stamp = rospy.Time.now()
            static_transform.header.frame_id = vehicle_base_frame
            static_transform.child_frame_id = sensor_echo_frames[sensor_index]
            static_transform.transform.translation.x = pose.position.x_val
            static_transform.transform.translation.y = -pose.position.y_val
            static_transform.transform.translation.z = -pose.position.z_val
            static_transform.transform.rotation.x = orientation.x_val
            static_transform.transform.rotation.y = orientation.y_val
            static_transform.transform.rotation.z = orientation.z_val
            static_transform.transform.rotation.w = orientation.w_val
            transform_list.append(static_transform)
            time.sleep(0.1)
            rospy.loginfo("Started static transform for echo sensor with ID " + sensor_name + ".")

        for sensor_index, sensor_name in enumerate(sensor_lidar_names):
            try:
                lidar_data = client.getLidarData(sensor_name, vehicle_name)
            except msgpackrpc.error.RPCError:
                rospy.logerr("LiDAR sensor '" + sensor_name + "' could not be found.")
                rospy.signal_shutdown('Sensor not found.')
                sys.exit()
            pose = lidar_data.pose
            orientation = pose.orientation.inverse()
            static_transform = TransformStamped()
            static_transform.header.stamp = rospy.Time.now()
            static_transform.header.frame_id = vehicle_base_frame
            static_transform.child_frame_id = sensor_lidar_frames[sensor_index]
            static_transform.transform.translation.x = pose.position.x_val
            static_transform.transform.translation.y = -pose.position.y_val
            static_transform.transform.translation.z = -pose.position.z_val
            static_transform.transform.rotation.x = orientation.x_val
            static_transform.transform.rotation.y = orientation.y_val
            static_transform.transform.rotation.z = orientation.z_val
            static_transform.transform.rotation.w = orientation.w_val
            transform_list.append(static_transform)
            time.sleep(0.1)
            rospy.loginfo("Started static transform for LiDAR sensor with ID " + sensor_name + ".")

        for sensor_index, sensor_name in enumerate(sensor_gpulidar_names):
            try:
                lidar_data = client.getGPULidarData(sensor_name, vehicle_name)
            except msgpackrpc.error.RPCError:
                rospy.logerr("GPU-LiDAR sensor '" + sensor_name + "' could not be found.")
                rospy.signal_shutdown('Sensor not found.')
                sys.exit()
            pose = lidar_data.pose
            orientation = pose.orientation.inverse()
            static_transform = TransformStamped()
            static_transform.header.stamp = rospy.Time.now()
            static_transform.header.frame_id = vehicle_base_frame
            static_transform.child_frame_id = sensor_gpulidar_frames[sensor_index]
            static_transform.transform.translation.x = pose.position.x_val
            static_transform.transform.translation.y = -pose.position.y_val
            static_transform.transform.translation.z = -pose.position.z_val
            static_transform.transform.rotation.x = orientation.x_val
            static_transform.transform.rotation.y = orientation.y_val
            static_transform.transform.rotation.z = orientation.z_val
            static_transform.transform.rotation.w = orientation.w_val
            transform_list.append(static_transform)
            time.sleep(0.1)
            rospy.loginfo("Started static transform for GPU-LiDAR sensor with ID " + sensor_name + ".")

        for sensor_index, sensor_name in enumerate(sensor_uwb_names):

            # Get uwb sensor data
            try:
                uwb_data = client.getUWBSensorData(sensor_name, vehicle_name)
                # timeStamp = uwb_data[0]

            except msgpackrpc.error.RPCError:
                rospy.logerr("UWB sensor '" + sensor_name + "' could not be found.")
                rospy.signal_shutdown('Sensor not found.')
                sys.exit()

            if (len(uwb_data) == 2):
                pose = uwb_data[1]
                static_transform = TransformStamped()
                static_transform.header.stamp = rospy.Time.now()
                static_transform.header.frame_id = vehicle_base_frame
                static_transform.child_frame_id = sensor_uwb_frames[sensor_index]
                static_transform.transform.translation.x = pose['position']['x_val']
                static_transform.transform.translation.y = -pose['position']['y_val']
                static_transform.transform.translation.z = -pose['position']['z_val']
                static_transform.transform.rotation.x = pose['orientation']['x_val']
                static_transform.transform.rotation.y = pose['orientation']['y_val']
                static_transform.transform.rotation.z = pose['orientation']['z_val']
                static_transform.transform.rotation.w = pose['orientation']['w_val']
                transform_list.append(static_transform)
                time.sleep(0.1)
                rospy.loginfo("Started static transform for UWB sensor with ID " + sensor_name + ".")
        left_position = None
        right_position = None
        for sensor_index, sensor_name in enumerate(sensor_camera_names):
            try:
                camera_data = client.simGetCameraInfo(sensor_name)
            except msgpackrpc.error.RPCError:
                rospy.logerr("camera sensor '" + sensor_name + "' could not be found.")
                rospy.signal_shutdown('Sensor not found.')
                sys.exit()
            pose = camera_data.pose
            orientation = pose.orientation.inverse()

            # camera frame
            static_transform = TransformStamped()
            static_transform.header.stamp = rospy.Time.now()
            static_transform.header.frame_id = vehicle_base_frame
            static_transform.child_frame_id = sensor_camera_frames[sensor_index]
            static_transform.transform.translation.x = pose.position.x_val
            static_transform.transform.translation.y = -pose.position.y_val
            static_transform.transform.translation.z = -pose.position.z_val
            static_transform.transform.rotation.x = orientation.x_val
            static_transform.transform.rotation.y = orientation.y_val
            static_transform.transform.rotation.z = orientation.z_val
            static_transform.transform.rotation.w = orientation.w_val
            transform_list.append(static_transform)

            # optical frame
            static_transform = TransformStamped()
            static_transform.header.stamp = rospy.Time.now()
            static_transform.header.frame_id = sensor_camera_frames[sensor_index]
            static_transform.child_frame_id = sensor_camera_optical_frames[sensor_index]
            static_transform.transform.translation.x = 0
            static_transform.transform.translation.y = 0
            static_transform.transform.translation.z = 0
            q_rot = quaternion_from_euler(-math.pi / 2, 0, -math.pi / 2)
            static_transform.transform.rotation.x = q_rot[0]
            static_transform.transform.rotation.y = q_rot[1]
            static_transform.transform.rotation.z = q_rot[2]
            static_transform.transform.rotation.w = q_rot[3]
            transform_list.append(static_transform)

            transform_list.append(static_transform)
            time.sleep(0.1)
            rospy.loginfo("Started static transform for camera sensor with ID " + sensor_name + ".")
            if sensor_stereo_enable == 1 and sensor_index == 0:
                left_position = [pose.position.x_val, -pose.position.y_val, -pose.position.z_val]
            elif sensor_stereo_enable == 1 and sensor_index == 1:
                right_position = [pose.position.x_val, -pose.position.y_val, -pose.position.z_val]
        if left_position != None and right_position != None:
            baseline = math.sqrt((left_position[0] - right_position[0]) ** 2 + (left_position[1] - right_position[1]) ** 2 + (left_position[2] - right_position[2]) ** 2)
        else:
            baseline = 0
        broadcaster.sendTransform(transform_list)

        airsim_publish(client, vehicle_name, pose_topic, pose_frame, tf_localisation_enable, carcontrol_enable, carcontrol_topic,
                       odometry_enable, odometry_topic, sensor_imu_enable,
                       sensor_imu_name, sensor_imu_topic, sensor_imu_frame, sensor_echo_names,
                       sensor_echo_topics, sensor_echo_frames, sensor_lidar_names,
                       sensor_lidar_toggle_groundtruth,
                       sensor_lidar_topics, sensor_lidar_segmentation_topics, sensor_lidar_frames,
                       sensor_gpulidar_names, sensor_gpulidar_topics,
                       sensor_gpulidar_frames, sensor_camera_names, sensor_camera_toggle_scene_mono,
                       sensor_camera_scene_quality, sensor_camera_toggle_segmentation,
                       sensor_camera_toggle_depth, sensor_camera_scene_topics,
                       sensor_camera_segmentation_topics, sensor_camera_depth_topics,
                       sensor_camera_frames, sensor_camera_optical_frames, sensor_camera_toggle_camera_info, sensor_camera_info_topics, sensor_stereo_enable, baseline,
                       object_names, objects_coordinates_local, object_topics)

    except rospy.ROSInterruptException:
        pass
