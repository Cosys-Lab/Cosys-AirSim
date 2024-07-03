#!/usr/bin/env python3

import cosysairsim as airsim
import rospy
import time
import math
from std_msgs.msg import String, Header, Float32, Float32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped, Point, Twist
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from airsimros.msg import StringArray
from sensor_msgs.msg import PointCloud2, PointField, Imu, CameraInfo
from nav_msgs.msg import Odometry
from uwb_msgs.msg import Diagnostics, Range, RangeArray
import numpy as np
import cv2
from sensor_msgs.msg import Image
import msgpackrpc
import sys
import tf
import tf2_msgs
from multiprocessing import Queue
import rosbag
from datetime import datetime


class PID(object):
    desired_speed = 0

    def __init__(self, kp, ki, kd, cur_time, topicname):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.pe = 0
        self.ie = 0
        self.de = 0
        self.time = cur_time
        self.integral = 0
        self.last_err = 0
        self.maxI = 1000000
        self.pid_queue = Queue(1)
        self.update_sub = rospy.Subscriber(topicname, Float32MultiArray, handle_input_command, self.pid_queue)

    def update_params(self):

        # print("yeet")
        try:
            speed_msg = self.pid_queue.get_nowait()
        except:
            return
        # print(speed_msg)
        self.kp = speed_msg.data[0]
        self.ki = speed_msg.data[1]
        self.kd = speed_msg.data[2]

    def get_p_error(self, error):
        PError = self.kp * error
        return PError

    def get_ie_error(self, error):
        self.integral += error
        if math.fabs(self.integral) > self.maxI:
            if self.integral > 0:
                self.integral = self.maxI
            else:
                self.integral = -self.maxI

        IError = self.ki * self.integral * self.time
        return IError

    def get_de_error(self, error):
        der = error - self.last_err
        self.last_err = error
        DError = self.kd * der / self.time
        return DError

    def get_error(self, error):
        return self.get_p_error(error) + self.get_ie_error(error) + self.get_de_error(error)


def pose_2_mat(p):
    q = p.orientation
    pos = p.position
    T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
    T[:3, 3] = np.array([pos.x, pos.y, pos.z])
    return T


def t_invert(t_in):
    R_in = t_in[:3, :3]
    t_in = t_in[:3, [-1]]
    R_out = R_in.T
    t_out = -np.matmul(R_out, t_in)
    return np.vstack((np.hstack((R_out, t_out)), np.array([0, 0, 0, 1])))


def mat_2_tf(mat, cur_time, frame, child_frame):
    t = TransformStamped()
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(mat)
    quat = tf.transformations.quaternion_from_euler(angles[0], angles[1], angles[2])
    t.header.stamp = cur_time
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


def handle_input_command(msg, args):
    # set the controls for car
    q = args
    try:
        q.put(msg)
    except:
        print("Input queue full")


def get_imu_ros_message(c, cur_sensor_name, cur_vehicle_name, cur_timestamp, cur_frame):
    imu_data = c.getImuData(cur_sensor_name, cur_vehicle_name)

    imu_msg = Imu()
    imu_msg.header.stamp = cur_timestamp
    imu_msg.header.frame_id = cur_frame

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

    return imu_msg


def get_camera_info_ros_message(cur_fov, cur_frame, cur_timestamp, cur_width, cur_height, stereo_enable, cur_baseline,
                                first_sensor):
    cam_info_msg = CameraInfo()
    cam_info_msg.header.frame_id = cur_frame
    cam_info_msg.header.stamp = cur_timestamp
    cam_info_msg.height = cur_height
    cam_info_msg.width = cur_width
    f = (cam_info_msg.width / 2.0) / math.tan(cur_fov * math.pi / 360)
    if stereo_enable and first_sensor:
        Tx = -f * cur_baseline
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
    return cam_info_msg


def get_depth_camera_ros_message(cur_camera_msg, cur_response):
    rgb_string = airsim.get_image_bytes(cur_response, "DepthPlanar")
    cur_camera_msg.encoding = "32FC1"
    cur_camera_msg.step = cur_response.width * 4
    cur_camera_msg.data = rgb_string
    return cur_camera_msg


def get_segmentation_camera_ros_message(cur_camera_msg, cur_response):
    rgb_string = airsim.get_image_bytes(cur_response, "Segmentation")
    cur_camera_msg.step = cur_response.width * 3
    cur_camera_msg.data = rgb_string
    return cur_camera_msg


def get_annotation_camera_ros_message(cur_camera_msg, cur_response):
    rgb_string = airsim.get_image_bytes(cur_response, "Annotation")
    cur_camera_msg.step = cur_response.width * 3
    cur_camera_msg.data = rgb_string
    return cur_camera_msg


def get_scene_camera_ros_message(cur_response, cur_timestamp, cur_frame, cur_cv_bridge,
                                 cur_sensor_camera_scene_quality, enable_mono):
    rgb_matrix = np.frombuffer(airsim.get_image_bytes(cur_response, "Scene"),
                               dtype=np.uint8).reshape(cur_response.height, cur_response.width, 3)
    if cur_sensor_camera_scene_quality > 0:
        rgb_matrix = cv2.cvtColor(rgb_matrix, cv2.COLOR_RGB2BGR)
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), cur_sensor_camera_scene_quality]
        result, img = cv2.imencode('.jpg', rgb_matrix, encode_params)
        rgb_matrix = cv2.imdecode(img, 1)
        rgb_matrix = cv2.cvtColor(rgb_matrix, cv2.COLOR_BGR2RGB)

    if enable_mono == 1:
        camera_msg = cur_cv_bridge.cv2_to_imgmsg(cv2.cvtColor(rgb_matrix, cv2.COLOR_RGB2GRAY), encoding="mono8")
    else:
        camera_msg = cur_cv_bridge.cv2_to_imgmsg(rgb_matrix, encoding="rgb8")
    camera_msg.header.stamp = cur_timestamp
    camera_msg.header.frame_id = cur_frame
    return camera_msg


def get_car_control_ros_message(c, cur_queue_input_command, cur_speed_pid, cur_odom, cur_vehicle_name,
                                cur_desired_rotation):
    # set the controls for car
    car_controls = airsim.CarControls()
    car_controls.is_manual_gear = True
    car_controls.manual_gear = 1

    try:
        # if queue_input_command.():
        #    msg.linear.x = 0
        msg = cur_queue_input_command.get_nowait()

        car_controls.is_manual_gear = True
        cur_speed_pid.desired_speed = msg.linear.x
        cur_desired_rotation = -msg.angular.z
    except:
        pass  # queue is empty

    cur_speed_pid.update_params()
    v = cur_odom.twist.twist.linear.x
    error = cur_speed_pid.desired_speed - v
    desired_accel = cur_speed_pid.get_error(error)
    # if abs(desired_accel) <= 0.05:
    #     desired_accel = 0
    wheelbase = 0.3
    if cur_desired_rotation == 0 or abs(v) <= 0.1:
        steer_angle = cur_desired_rotation
    else:
        radius = v / cur_desired_rotation
        steer_angle = math.atan(wheelbase / radius)
    # steer_angle = cur_desired_rotation

    if abs(steer_angle) <= 0.05:
        steer_angle = 0
    car_controls.steering = steer_angle

    if cur_speed_pid.desired_speed == 0 and abs(v) <= 0.1 and cur_desired_rotation == 0:
        car_controls.brake = 0.1
        car_controls.throttle = 0.0
    else:
        car_controls.throttle = desired_accel
        car_controls.brake = 0.0

    c.setCarControls(car_controls, cur_vehicle_name)

    return cur_speed_pid.desired_speed, v, error, cur_desired_rotation


def get_all_objects_ros_path_message(c, cur_timestamp, cur_first_message, cur_object_poses_all_once, cur_map_frame,
                                     cur_object_poses_all_coordinates_local):
    if not cur_object_poses_all_once or (cur_object_poses_all_once and cur_first_message):
        object_path = Path()
        object_path.header.stamp = cur_timestamp
        object_path.header.frame_id = cur_map_frame

        cur_object_names = c.simListInstanceSegmentationObjects()
        if cur_object_poses_all_coordinates_local == 1:
            cur_object_poses = c.simListInstanceSegmentationPoses(True)
        else:
            cur_object_poses = c.simListInstanceSegmentationPoses(False)
        for index, object_name in enumerate(cur_object_names):
            cur_object_pose = cur_object_poses[index]
            if not np.isnan(cur_object_pose.position.x_val):
                cur_pos = cur_object_pose.position
                cur_orientation = cur_object_pose.orientation.inverse()

                object_pose = PoseStamped()
                object_pose.pose.position.x = cur_pos.x_val
                object_pose.pose.position.y = -cur_pos.y_val
                object_pose.pose.position.z = -cur_pos.z_val
                object_pose.pose.orientation.w = cur_orientation.w_val
                object_pose.pose.orientation.x = cur_orientation.x_val
                object_pose.pose.orientation.y = cur_orientation.y_val
                object_pose.pose.orientation.z = cur_orientation.z_val
                object_pose.header.frame_id = object_name
                object_pose.header.stamp = cur_timestamp
                object_path.poses.append(object_pose)
        return object_path
    else:
        return None


def get_object_pose_ros_message(c, object_pose_individual_coordinates_local, cur_object_name, cur_warning_issued,
                                cur_rospy, cur_timestamp, cur_map_frame):
    if object_pose_individual_coordinates_local == 1:
        cur_pose = c.simGetObjectPose(cur_object_name, True)
    else:
        cur_pose = c.simGetObjectPose(cur_object_name, False)
    if np.isnan(cur_pose.position.x_val):
        if cur_warning_issued is False:
            cur_rospy.logwarn("Object '" + cur_object_name + "' could not be found.")
            warning_issued_result = True
        return None, warning_issued_result
    else:
        warning_issued_result = False
        cur_pos = cur_pose.position
        cur_orientation = cur_pose.orientation.inverse()

        object_pose = PoseStamped()
        object_pose.pose.position.x = cur_pos.x_val
        object_pose.pose.position.y = -cur_pos.y_val
        object_pose.pose.position.z = -cur_pos.z_val
        object_pose.pose.orientation.w = cur_orientation.w_val
        object_pose.pose.orientation.x = cur_orientation.x_val
        object_pose.pose.orientation.y = cur_orientation.y_val
        object_pose.pose.orientation.z = cur_orientation.z_val
        object_pose.header.stamp = cur_timestamp
        object_pose.header.seq = 1
        object_pose.header.frame_id = cur_map_frame

        return object_pose, warning_issued_result


def get_wifi_ros_message(c, cur_vehicle_name, cur_rospy, cur_timestamp):
    range_arrays = []
    wifi_data = c.getWifiData(vehicle_name=cur_vehicle_name)

    # Sanity check
    if len(wifi_data.wr_anchorPosX) != len(wifi_data.wr_anchorPosY) or \
            len(wifi_data.wr_anchorPosX) != len(wifi_data.wr_anchorPosZ) or \
            len(wifi_data.wr_anchorPosX) != len(wifi_data.wr_distance) or \
            len(wifi_data.wr_anchorPosX) != len(wifi_data.wr_rssi) or \
            len(wifi_data.wr_anchorPosX) != len(wifi_data.wr_time_stamp) or \
            len(wifi_data.wr_anchorPosX) != len(wifi_data.wr_anchorId) or \
            len(wifi_data.wr_anchorPosX) != len(wifi_data.wr_valid_range):
        cur_rospy.logerr("Wifi sensor wr lengths do not match")
        cur_rospy.signal_shutdown('Packet error.')
        sys.exit()
    if len(wifi_data.wra_ranges) != len(wifi_data.wra_tagId) or \
            len(wifi_data.wra_ranges) != len(wifi_data.wra_tagPosX) or \
            len(wifi_data.wra_ranges) != len(wifi_data.wra_tagPosY) or \
            len(wifi_data.wra_ranges) != len(wifi_data.wra_tagPosZ):
        cur_rospy.logerr("Wifi sensor wra lengths do not match")
        cur_rospy.signal_shutdown('Packet error.')
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
        wr_anchorPosX += list(np.array(wifi_data.wr_anchorPosX)[uniqueRangeIds])
        wr_anchorPosY += list(np.array(wifi_data.wr_anchorPosY)[uniqueRangeIds])
        wr_anchorPosZ += list(np.array(wifi_data.wr_anchorPosZ)[uniqueRangeIds])
        wr_time_stamp += list(np.array(wifi_data.wr_time_stamp)[uniqueRangeIds])
        wr_anchorId += list(np.array(wifi_data.wr_anchorId)[uniqueRangeIds])
        wr_valid_range += list(np.array(wifi_data.wr_valid_range)[uniqueRangeIds])

        wra_ranges.append([])
        wra_ranges[rangeIdx] = list(range(wra_ranges_idx_start, len(wr_anchorPosX)))
        for arange in uniqueRangeIds:
            currentRanges = np.array(wifi_data.wr_anchorId)[ranges]
            currentRssi = np.array(wifi_data.wr_rssi)[ranges]
            currentDistance = np.array(wifi_data.wr_distance)[ranges]
            currentRssiFiltered = currentRssi * list(
                map(int, currentRanges == wifi_data.wr_anchorId[arange]))
            maxRssi = max(currentRssiFiltered)
            maxRssiIdx = currentRssiFiltered.argmax()

            wr_distance.append(currentDistance[maxRssiIdx])
            wr_rssi.append(maxRssi)

        # for arange in uniqueRangeIds:

        idx_offset += len(ranges)

    for wra_idx in range(0, len(wifi_data.wra_tagId)):
        range_array = RangeArray()
        range_array.tagid = str(wifi_data.wra_tagId[wra_idx])
        range_array.tag_position = Point(wifi_data.wra_tagPosX[wra_idx], wifi_data.wra_tagPosY[wra_idx],
                                         wifi_data.wra_tagPosZ[wra_idx])
        range_array.header.stamp = cur_timestamp

        # for wr_id in range(0, len(wr_time_stamp)):
        for wr_id in wra_ranges[wra_idx]:
            diag = Diagnostics()
            diag.rssi = wr_rssi[wr_id]

            rang = Range()
            # rang.stamp = wr_time_stamp[wr_id]
            rang.stamp = cur_timestamp
            rang.anchorid = str(wr_anchorId[wr_id])
            rang.anchor_position = Point(wr_anchorPosX[wr_id], wr_anchorPosY[wr_id], wr_anchorPosZ[wr_id])
            rang.valid_range = wr_valid_range[wr_id]
            rang.distance = wr_distance[wr_id]
            rang.diagnostics = diag

            range_array.ranges.append(rang)
            range_arrays.append(range_array)

    return range_arrays


def get_uwb_ros_message(c, cur_vehicle_name, cur_rospy, cur_timestamp):
    range_arrays = []

    cur_uwb_data = c.getUWBData(vehicle_name=cur_vehicle_name)

    # Sanity check
    if len(cur_uwb_data.mur_anchorPosX) != len(cur_uwb_data.mur_anchorPosY) or \
            len(cur_uwb_data.mur_anchorPosX) != len(cur_uwb_data.mur_anchorPosZ) or \
            len(cur_uwb_data.mur_anchorPosX) != len(cur_uwb_data.mur_distance) or \
            len(cur_uwb_data.mur_anchorPosX) != len(cur_uwb_data.mur_rssi) or \
            len(cur_uwb_data.mur_anchorPosX) != len(cur_uwb_data.mur_time_stamp) or \
            len(cur_uwb_data.mur_anchorPosX) != len(cur_uwb_data.mur_anchorId) or \
            len(cur_uwb_data.mur_anchorPosX) != len(cur_uwb_data.mur_valid_range):
        cur_rospy.logerr("UWB sensor mur lengths do not match")
        cur_rospy.signal_shutdown('Packet error.')
        sys.exit()
    if len(cur_uwb_data.mura_ranges) != len(cur_uwb_data.mura_tagId) or \
            len(cur_uwb_data.mura_ranges) != len(cur_uwb_data.mura_tagPosX) or \
            len(cur_uwb_data.mura_ranges) != len(cur_uwb_data.mura_tagPosY) or \
            len(cur_uwb_data.mura_ranges) != len(cur_uwb_data.mura_tagPosZ):
        cur_rospy.logerr("UWB sensor mura lengths do not match")
        cur_rospy.signal_shutdown('Packet error.')
        sys.exit()

    cur_uwb_data.mur_anchorId = np.array(cur_uwb_data.mur_anchorId)

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

    for rangeIdx, ranges in enumerate(cur_uwb_data.mura_ranges):
        u, uniqueRangeIds = np.unique(cur_uwb_data.mur_anchorId[ranges], return_index=True)
        uniqueRangeIds += idx_offset

        mura_ranges_idx_start = len(mur_anchorPosX)
        mur_anchorPosX += list(np.array(cur_uwb_data.mur_anchorPosX)[uniqueRangeIds])
        mur_anchorPosY += list(np.array(cur_uwb_data.mur_anchorPosY)[uniqueRangeIds])
        mur_anchorPosZ += list(np.array(cur_uwb_data.mur_anchorPosZ)[uniqueRangeIds])
        mur_time_stamp += list(np.array(cur_uwb_data.mur_time_stamp)[uniqueRangeIds])
        mur_anchorId += list(np.array(cur_uwb_data.mur_anchorId)[uniqueRangeIds])
        mur_valid_range += list(np.array(cur_uwb_data.mur_valid_range)[uniqueRangeIds])

        mura_ranges.append([])
        mura_ranges[rangeIdx] = list(range(mura_ranges_idx_start, len(mur_anchorPosX)))
        for arange in uniqueRangeIds:
            currentRanges = np.array(cur_uwb_data.mur_anchorId)[ranges]
            currentRssi = np.array(cur_uwb_data.mur_rssi)[ranges]
            currentDistance = np.array(cur_uwb_data.mur_distance)[ranges]
            currentRssiFiltered = currentRssi * list(
                map(int, currentRanges == cur_uwb_data.mur_anchorId[arange]))
            maxRssi = max(currentRssiFiltered)
            maxRssiIdx = currentRssiFiltered.argmax()

            mur_distance.append(currentDistance[maxRssiIdx])
            mur_rssi.append(maxRssi)

        # for arange in uniqueRangeIds:
        idx_offset += len(ranges)

    for mura_idx in range(0, len(cur_uwb_data.mura_tagId)):
        range_array = RangeArray()
        range_array.tagid = str(cur_uwb_data.mura_tagId[mura_idx])
        range_array.tag_position = Point(cur_uwb_data.mura_tagPosX[mura_idx], cur_uwb_data.mura_tagPosY[mura_idx],
                                         cur_uwb_data.mura_tagPosZ[mura_idx])
        range_array.header.stamp = cur_timestamp

        # for mur_id in range(0, len(mur_time_stamp)):
        for mur_id in mura_ranges[mura_idx]:
            diag = Diagnostics()
            diag.rssi = mur_rssi[mur_id]

            rang = Range()
            # rang.stamp = mur_time_stamp[mur_id]
            rang.stamp = cur_timestamp
            rang.anchorid = str(mur_anchorId[mur_id]).split(":")[-1]
            rang.anchor_position = Point(mur_anchorPosX[mur_id], mur_anchorPosY[mur_id],
                                         mur_anchorPosZ[mur_id])
            rang.valid_range = mur_valid_range[mur_id]
            rang.distance = mur_distance[mur_id]
            rang.diagnostics = diag

            range_array.ranges.append(rang)
            range_arrays.append(range_array)

    return range_arrays


def get_gpulidar_ros_message(c, cur_sensor_name, cur_vehicle_name, cur_last_timestamp, cur_fields_lidar,
                             cur_sensor_gpulidar_frame, cur_timestamp):
    cur_lidar_data = c.getGPULidarData(cur_sensor_name, cur_vehicle_name)

    if cur_lidar_data.time_stamp != cur_last_timestamp:
        if len(cur_lidar_data.point_cloud) < 5:
            last_timestamp_return = cur_lidar_data.time_stamp
            return None, last_timestamp_return
        else:
            last_timestamp_return = cur_lidar_data.time_stamp

            pcloud = PointCloud2()
            points = np.array(cur_lidar_data.point_cloud, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 5), 5))
            points_x = points[:, 0]
            points_y = -points[:, 1]
            points_z = -points[:, 2]
            points_rgb = points[:, 3].astype('uint32')
            points_intensity = points[:, 4]
            points_list = [points_x, points_y, points_z, points_rgb, points_intensity]
            points_list_transposed = [list(i) for i in zip(*points_list)]
            pcloud.header.frame_id = cur_sensor_gpulidar_frame
            pcloud.header.stamp = cur_timestamp
            pcloud = pc2.create_cloud(pcloud.header, cur_fields_lidar, points_list_transposed)
            return pcloud, last_timestamp_return
    else:
        return None, None


def get_lidar_ros_message(c, cur_sensor_name, cur_vehicle_name, cur_last_timestamp, cur_sensor_lidar_frame,
                          cur_timestamp, cur_sensor_lidar_toggle_segmentation):
    cur_lidar_data = c.getLidarData(cur_sensor_name, cur_vehicle_name)

    if cur_lidar_data.time_stamp != cur_last_timestamp:
        if len(cur_lidar_data.point_cloud) < 4:
            last_timestamp_return = cur_lidar_data.time_stamp
            return None, None, last_timestamp_return
        else:
            last_timestamp_return = cur_lidar_data.time_stamp

            pcloud = PointCloud2()
            points = np.array(cur_lidar_data.point_cloud, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 3), 3))
            points = points * np.array([1, -1, -1])
            cloud = points.tolist()
            pcloud.header.frame_id = cur_sensor_lidar_frame
            pcloud.header.stamp = cur_timestamp
            pcloud = pc2.create_cloud_xyz32(pcloud.header, cloud)

            if cur_sensor_lidar_toggle_segmentation == 1:
                labels = np.array(cur_lidar_data.groundtruth, dtype=np.dtype('U'))
                groundtruth = StringArray()
                groundtruth.data = labels.tolist()
                groundtruth.header.frame_id = cur_sensor_lidar_frame
                groundtruth.header.stamp = cur_timestamp
            else:
                groundtruth = None

            return pcloud, groundtruth, last_timestamp_return
    else:
        return None, None, None


def get_echo_ros_message(c, cur_sensor_name, cur_vehicle_name, cur_last_timestamp,
                         cur_fields_echo, cur_fields_echo_passive, cur_sensor_echo_frame, cur_timestamp,
                         passive_enable):
    cur_echo_data = c.getEchoData(cur_sensor_name, cur_vehicle_name)
    if cur_echo_data.time_stamp != cur_last_timestamp:
        if len(cur_echo_data.point_cloud) < 7 and len(cur_echo_data.passive_beacons_point_cloud) < 10:
            last_timestamp_return = cur_timestamp
            return None, None, last_timestamp_return, None, None
        else:
            last_timestamp_return = cur_timestamp
            header = Header()
            header.frame_id = cur_sensor_echo_frame

            if len(cur_echo_data.point_cloud) > 5:
                points = np.array(cur_echo_data.point_cloud, dtype=np.dtype('f4'))
                points = np.reshape(points, (int(points.shape[0] / 6), 6))
                points = points * np.array([1, -1, -1, 1, 1, 1])
                points_list = points.tolist()
                pcloud = pc2.create_cloud(header, cur_fields_echo, points_list)
                pcloud.header.stamp = cur_timestamp

                labels = np.array(cur_echo_data.groundtruth, dtype=np.dtype('U'))
                groundtruth = StringArray()
                groundtruth.data = labels.tolist()
                groundtruth.header.frame_id = cur_sensor_echo_frame
                groundtruth.header.stamp = cur_timestamp
            else:
                pcloud = None
                groundtruth = None

            if passive_enable:
                if len(cur_echo_data.passive_beacons_point_cloud) > 8:
                    pointsp = np.array(cur_echo_data.passive_beacons_point_cloud, dtype=np.dtype('f4'))
                    pointsp = np.reshape(pointsp, (int(pointsp.shape[0] / 9), 9))
                    pointsp = pointsp * np.array([1, -1, -1, 1, 1, 1, 1, -1, -1])
                    pointsp_list = pointsp.tolist()
                    pcloud_passive = pc2.create_cloud(header, cur_fields_echo_passive, pointsp_list)
                    pcloud_passive.header.stamp = cur_timestamp

                    labelsp = np.array(cur_echo_data.passive_beacons_groundtruth, dtype=np.dtype('U'))
                    groundtruth_passive = StringArray()
                    groundtruth_passive.data = labelsp.tolist()
                    groundtruth_passive.header.frame_id = cur_sensor_echo_frame
                    groundtruth_passive.header.stamp = cur_timestamp
                else:
                    pcloud_passive = None
                    groundtruth_passive = None
        return pcloud, groundtruth, last_timestamp_return, pcloud_passive, groundtruth_passive
    else:
        return None, None, None, None, None


def airsim_publish(client, use_route, route_rosbag, merged_rosbag, generate_gt_map, saved_static_tf,
                   vehicle_name, pose_topic, map_frame, odom_frame, tf_odom_enable,
                   pose_offset_x, pose_offset_y, pose_offset_z,
                   carcontrol_enable, carcontrol_topic, odometry_enable, odometry_topic,
                   sensor_imu_enable, sensor_imu_name, sensor_imu_topic,
                   sensor_imu_frame, sensor_echo_names,
                   sensor_echo_topics, sensor_echo_segmentation_topics, sensor_echo_frames,
                   sensor_echo_toggle_passive, sensor_echo_passive_topics, sensor_echo_passive_segmentation_topics,
                   sensor_lidar_names, sensor_lidar_toggle_segmentation,
                   sensor_lidar_topics, sensor_lidar_segmentation_topics, sensor_lidar_frames,
                   sensor_gpulidar_names, sensor_gpulidar_topics,
                   sensor_gpulidar_frames, sensor_camera_names, sensor_camera_toggle_scene_mono,
                   sensor_camera_scene_quality, sensor_camera_toggle_segmentation,
                   sensor_camera_toggle_depth, sensor_camera_toggle_annotation, sensor_camera_annotation_layers,
                   sensor_camera_scene_topics, sensor_camera_segmentation_topics, sensor_camera_depth_topics,
                   sensor_camera_annotation_topics, sensor_camera_optical_frames, sensor_camera_toggle_camera_info,
                   sensor_camera_info_topics, sensor_stereo_enable, baseline,
                   object_poses_all, object_poses_all_coordinates_local, object_poses_all_once, object_poses_all_topic,
                   object_poses_individual_names, object_poses_individual_coordinates_local,
                   object_poses_individual_topics, sensor_uwb_names, sensor_uwb_topic,
                   sensor_wifi_names, sensor_wifi_topic):
    if not use_route:
        rate = rospy.Rate(ros_rate)

        speed_pid = PID(1, 0, 0.001, 1.0 / ros_rate, "/speed_pid")
        # steering_pid = PID(0.0, 0.3, 0.05, 1.0/ros_rate, "/steer_pid")

        last_timestamps = {}
        warning_issued = {}
        pose_publisher = rospy.Publisher(pose_topic, PoseStamped, queue_size=1)
        if sensor_imu_enable:
            imu_publisher = rospy.Publisher(sensor_imu_topic, Imu, queue_size=1)
        if odometry_enable:
            odom_publisher = rospy.Publisher(odometry_topic, Odometry, queue_size=1)
        if tf_odom_enable:
            tf_br = tf2_ros.TransformBroadcaster()
        objectpose_publishers = {}
        pointcloud_publishers = {}
        string_segmentation_publishers = {}
        image_publishers = {}
        cameraInfo_objects = {}
        for cur_sensor_index, cur_sensor_name in enumerate(sensor_echo_names):
            pointcloud_publishers[cur_sensor_name] = rospy.Publisher(sensor_echo_topics[cur_sensor_index], PointCloud2,
                                                                     queue_size=2)
            string_segmentation_publishers[cur_sensor_name] = \
                rospy.Publisher(sensor_echo_segmentation_topics[cur_sensor_index], StringArray, queue_size=1)
            last_timestamps[cur_sensor_name] = None
            if sensor_echo_toggle_passive[cur_sensor_index] == 1:
                pointcloud_publishers[cur_sensor_name + '_passive'] = \
                    rospy.Publisher(sensor_echo_passive_topics[cur_sensor_index], PointCloud2, queue_size=2)
                last_timestamps[cur_sensor_name + '_passive'] = None
                string_segmentation_publishers[cur_sensor_name + '_passive'] = \
                    rospy.Publisher(sensor_echo_passive_segmentation_topics[cur_sensor_index], StringArray,
                                    queue_size=1)
        for cur_sensor_index, cur_sensor_name in enumerate(sensor_lidar_names):
            pointcloud_publishers[cur_sensor_name] = rospy.Publisher(sensor_lidar_topics[cur_sensor_index], PointCloud2,
                                                                     queue_size=2)
            last_timestamps[cur_sensor_name] = None
            if sensor_lidar_toggle_segmentation[cur_sensor_index] == 1:
                string_segmentation_publishers[cur_sensor_name] = \
                    rospy.Publisher(sensor_lidar_segmentation_topics[cur_sensor_index], StringArray, queue_size=1)
        for cur_sensor_index, cur_sensor_name in enumerate(sensor_gpulidar_names):
            pointcloud_publishers[cur_sensor_name] = rospy.Publisher(sensor_gpulidar_topics[cur_sensor_index],
                                                                     PointCloud2, queue_size=2)
            last_timestamps[cur_sensor_name] = None

        if len(sensor_uwb_names) > 0:
            uwb_range_array_publisher = rospy.Publisher(sensor_uwb_topic, RangeArray, queue_size=10)

        if len(sensor_wifi_names) > 0:
            wifi_range_array_publisher = rospy.Publisher(sensor_wifi_topic, RangeArray, queue_size=10)

        for cur_sensor_index, cur_sensor_name in enumerate(sensor_camera_names):
            image_publishers[cur_sensor_name + '_scene'] = \
                rospy.Publisher(sensor_camera_scene_topics[cur_sensor_index], Image, queue_size=2)
            if sensor_camera_toggle_segmentation[cur_sensor_index] == 1:
                image_publishers[cur_sensor_name + '_segmentation'] = \
                    rospy.Publisher(sensor_camera_segmentation_topics[cur_sensor_index], Image, queue_size=2)
            if sensor_camera_toggle_depth[cur_sensor_index] == 1:
                image_publishers[cur_sensor_name + '_depth'] = \
                    rospy.Publisher(sensor_camera_depth_topics[cur_sensor_index], Image, queue_size=2)
            if sensor_camera_toggle_annotation[cur_sensor_index] == 1:
                image_publishers[cur_sensor_name + '_annotation'] = \
                    rospy.Publisher(sensor_camera_annotation_topics[cur_sensor_index], Image, queue_size=2)
            if sensor_camera_toggle_camera_info[cur_sensor_index] == 1:
                image_publishers[cur_sensor_name + '_cameraInfo'] = \
                    rospy.Publisher(sensor_camera_info_topics[cur_sensor_index], CameraInfo, queue_size=2)
                cameraInfo_objects[cur_sensor_name] = client.simGetCameraInfo(cur_sensor_name, vehicle_name)

        if object_poses_all:
            object_path_publisher = rospy.Publisher(object_poses_all_topic, Path, queue_size=1)
        else:
            for object_index, object_name in enumerate(object_poses_individual_names):
                objectpose_publishers[object_name] = rospy.Publisher(object_poses_individual_topics[object_index],
                                                                     PoseStamped, queue_size=1)
                warning_issued[object_name] = False

        if carcontrol_enable == 1:
            print("Control of car enabled")
            desired_rotation = 0
            client.enableApiControl(True, vehicle_name=vehicle_name)
            queue_input_command = Queue(1)
            rospy.Subscriber(carcontrol_topic, Twist, handle_input_command, queue_input_command)
            request_speed_publisher = rospy.Publisher("/req_speed", Float32, queue_size=1)
            real_speed_publisher = rospy.Publisher("/real_speed", Float32, queue_size=1)
            error_speed_publisher = rospy.Publisher("/speed_error", Float32, queue_size=1)

        rospy.loginfo("Started publishers...")
    else:
        rospy.loginfo("Reading route...")
        route = rosbag.Bag(route_rosbag)
        output = rosbag.Bag(merged_rosbag, 'w')
        rospy.loginfo("Route retrieved!")

        rospy.loginfo("Started publishers...")
        rospy.logwarn("Ensure focus is on the screen of AirSim simulator to allow auto configuration!")
        rospy.logdebug(str(route.get_type_and_topic_info()))

        last_timestamps = {}
        warning_issued = {}
        for cur_sensor_index, cur_sensor_name in enumerate(sensor_echo_names):
            last_timestamps[cur_sensor_name] = None
        for cur_sensor_index, cur_sensor_name in enumerate(sensor_lidar_names):
            last_timestamps[cur_sensor_name] = None
        for cur_sensor_index, cur_sensor_name in enumerate(sensor_gpulidar_names):
            last_timestamps[cur_sensor_name] = None
        for object_index, object_name in enumerate(object_poses_individual_names):
            warning_issued[object_name] = False

        cameraInfo_objects = {}
        for cur_sensor_index, cur_sensor_name in enumerate(sensor_camera_names):
            if sensor_camera_toggle_camera_info[cur_sensor_index] == 1:
                cameraInfo_objects[cur_sensor_name] = client.simGetCameraInfo(cur_sensor_name, vehicle_name)

        pose_count = route.get_message_count('/' + pose_topic)
        pose_index = 1
        period = 1 / ros_rate
        tolerance = 0.05 * period
        last_time = 0
        first_timestamp = None

    first_message = True

    requests = []
    response_locations = {}
    response_index = 0
    toggle_mono = False
    for cur_sensor_index, cur_sensor_name in enumerate(sensor_camera_names):
        toggle_mono = True
        response_locations[cur_sensor_name + '_scene'] = response_index
        response_index += 1
        requests.append(airsim.ImageRequest(cur_sensor_name, airsim.get_camera_type("Scene"),
                                            airsim.is_pixels_as_float("Scene"), False))
        if sensor_camera_toggle_segmentation[cur_sensor_index] == 1:
            requests.append(airsim.ImageRequest(cur_sensor_name, airsim.get_camera_type("Segmentation"),
                                                airsim.is_pixels_as_float("Segmentation"), False))
            response_locations[cur_sensor_name + '_segmentation'] = response_index
            response_index += 1
        if sensor_camera_toggle_depth[cur_sensor_index] == 1:
            requests.append(airsim.ImageRequest(cur_sensor_name, airsim.get_camera_type("DepthPlanar"),
                                                airsim.is_pixels_as_float("DepthPlanar"), False))
            response_locations[cur_sensor_name + '_depth'] = response_index
            response_index += 1
        if sensor_camera_toggle_annotation[cur_sensor_index] == 1:
            requests.append(airsim.ImageRequest(cur_sensor_name, airsim.get_camera_type("Annotation"),
                                                airsim.is_pixels_as_float("Annotation"), False,
                                                sensor_camera_annotation_layers[cur_sensor_index]))
            response_locations[cur_sensor_name + '_annotation'] = response_index
            response_index += 1

    cv_bridge = None
    if toggle_mono:
        from cv_bridge import CvBridge
        cv_bridge = CvBridge()

    fields_echo = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('a', 12, PointField.FLOAT32, 1),
        PointField('d', 16, PointField.FLOAT32, 1),
        PointField('r', 16, PointField.FLOAT32, 1)
    ]

    fields_echo_passive = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('a', 12, PointField.FLOAT32, 1),
        PointField('d', 16, PointField.FLOAT32, 1),
        PointField('r', 20, PointField.FLOAT32, 1),
        PointField('xd', 24, PointField.FLOAT32, 1),
        PointField('yd', 28, PointField.FLOAT32, 1),
        PointField('zd', 32, PointField.FLOAT32, 1),
    ]

    fields_lidar = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('rgb', 12, PointField.UINT32, 1),
        PointField('intensity', 16, PointField.FLOAT32, 1)
    ]

    if use_route:
        for topic, msg, t in route.read_messages(topics=['/' + pose_topic, 'tf_static']):

            if rospy.is_shutdown():
                break

            ros_timestamp = t
            if first_timestamp is None:
                first_timestamp = ros_timestamp
            timestamp = msg.header.stamp

            if topic == "tf_static":
                saved_static_tf.transforms = saved_static_tf.transforms + msg.transforms

            elif topic == '/' + pose_topic:
                elapsedTime = t.to_sec() - last_time
                if elapsedTime + tolerance >= period:
                    client.simContinueForTime(period)
                    last_time = t.to_sec()
                    cur_position = airsim.Vector3r(msg.pose.position.x, -msg.pose.position.y, -msg.pose.position.z)
                    cur_orientation = airsim.Quaternionr(msg.pose.orientation.x, msg.pose.orientation.y,
                                                         msg.pose.orientation.z, msg.pose.orientation.w).inverse()
                    cur_orientation = airsim.Quaternionr(float(cur_orientation.x_val), float(cur_orientation.y_val),
                                                         float(cur_orientation.z_val), float(cur_orientation.w_val))
                    client.simSetVehiclePose(airsim.Pose(cur_position, cur_orientation), True, vehicle_name)

                    rospy.loginfo("Setting vehicle pose " + str(pose_index) + ' of ' + str(pose_count)
                                  + ' to record sensor data...')

                    camera_responses = client.simGetImages(requests, vehicle_name)
                    for cur_sensor_index, cur_sensor_name in enumerate(sensor_camera_names):
                        response = camera_responses[response_locations[cur_sensor_name + '_scene']]
                        if response.width == 0 and response.height == 0:
                            rospy.logwarn("Camera '" + cur_sensor_name + "' could not retrieve scene image.")
                        else:
                            camera_msg = (
                                get_scene_camera_ros_message(response, timestamp,
                                                             sensor_camera_optical_frames[cur_sensor_index],
                                                             cv_bridge,
                                                             sensor_camera_scene_quality[cur_sensor_index],
                                                             sensor_camera_toggle_scene_mono[cur_sensor_index]))
                            output.write(sensor_camera_scene_topics[cur_sensor_index], camera_msg, t=ros_timestamp)
                        if sensor_camera_toggle_segmentation[cur_sensor_index] == 1:
                            response = camera_responses[response_locations[cur_sensor_name + '_segmentation']]
                            if response.width == 0 and response.height == 0:
                                rospy.logwarn("Camera '" + cur_sensor_name + "' could not retrieve segmentation image.")
                            else:
                                camera_msg = get_segmentation_camera_ros_message(camera_msg, response)
                                output.write(sensor_camera_segmentation_topics[cur_sensor_index], camera_msg,
                                             t=ros_timestamp)
                        if sensor_camera_toggle_depth[cur_sensor_index] == 1:
                            response = camera_responses[response_locations[cur_sensor_name + '_depth']]
                            if response.width == 0 and response.height == 0:
                                rospy.logwarn("Camera '" + cur_sensor_name + "' could not retrieve depth image.")
                            else:
                                camera_msg = get_depth_camera_ros_message(camera_msg, response)
                                output.write(sensor_camera_depth_topics[cur_sensor_index], camera_msg,
                                             t=ros_timestamp)
                        if sensor_camera_toggle_annotation[cur_sensor_index] == 1:
                            response = camera_responses[response_locations[cur_sensor_name + '_annotation']]
                            if response.width == 0 and response.height == 0:
                                rospy.logwarn("Camera '" + cur_sensor_name + "' could not retrieve annotation image.")
                            else:
                                camera_msg = get_annotation_camera_ros_message(camera_msg, response)
                                output.write(sensor_camera_annotation_topics[cur_sensor_index], camera_msg,
                                             t=ros_timestamp)
                        if sensor_camera_toggle_camera_info[cur_sensor_index] == 1:
                            if cur_sensor_index == 1:
                                first_sensor = True
                            else:
                                first_sensor = False
                            cam_info_msg = get_camera_info_ros_message(cameraInfo_objects[cur_sensor_name].fov,
                                                                       sensor_camera_optical_frames[cur_sensor_index],
                                                                       timestamp, response.width, response.height,
                                                                       sensor_stereo_enable, baseline, first_sensor)
                            output.write(sensor_camera_info_topics[cur_sensor_index], cam_info_msg, t=ros_timestamp)

                    for cur_sensor_index, cur_sensor_name in enumerate(sensor_echo_names):
                        pcloud, groundtruth, last_timestamp_return, pcloud_passive, groundtruth_passive = (
                            get_echo_ros_message(client, cur_sensor_name, vehicle_name,
                                                 last_timestamps[cur_sensor_name], fields_echo, fields_echo_passive,
                                                 sensor_echo_frames[cur_sensor_index], timestamp,
                                                 sensor_echo_toggle_passive[cur_sensor_index]))
                        if last_timestamp_return is not None:
                            last_timestamps[cur_sensor_name] = last_timestamp_return
                        if pcloud is not None:
                            output.write(sensor_echo_topics[cur_sensor_index], pcloud, t=ros_timestamp)

                        if groundtruth is not None:
                            output.write(sensor_echo_segmentation_topics[cur_sensor_index], groundtruth,
                                         t=ros_timestamp)
                        if sensor_echo_toggle_passive[cur_sensor_index]:
                            if pcloud_passive is not None:
                                output.write(sensor_echo_passive_topics[cur_sensor_index], pcloud_passive,
                                             t=ros_timestamp)

                            if groundtruth_passive is not None:
                                output.write(sensor_echo_passive_segmentation_topics[cur_sensor_index],
                                             groundtruth_passive, t=ros_timestamp)

                    for cur_sensor_index, cur_sensor_name in enumerate(sensor_lidar_names):
                        seg_enable = sensor_lidar_toggle_segmentation[cur_sensor_index]
                        cur_frame = sensor_lidar_frames[cur_sensor_index]
                        pcloud, groundtruth, last_timestamp_return = get_lidar_ros_message(client, cur_sensor_name,
                                                                                           vehicle_name,
                                                                                           last_timestamps[
                                                                                               cur_sensor_name],
                                                                                           cur_frame,
                                                                                           timestamp,
                                                                                           seg_enable)
                        if last_timestamp_return is not None:
                            last_timestamps[cur_sensor_name] = last_timestamp_return
                        if pcloud is not None:
                            output.write(sensor_lidar_topics[cur_sensor_index], pcloud, t=ros_timestamp)
                        if sensor_lidar_toggle_segmentation[cur_sensor_index] == 1:
                            if groundtruth is not None:
                                output.write(sensor_lidar_segmentation_topics[cur_sensor_index], groundtruth,
                                             t=ros_timestamp)

                    for cur_sensor_index, cur_sensor_name in enumerate(sensor_gpulidar_names):
                        pcloud, last_timestamp_return = get_gpulidar_ros_message(client, cur_sensor_name, vehicle_name,
                                                                                 last_timestamps[cur_sensor_name],
                                                                                 fields_lidar,
                                                                                 sensor_gpulidar_frames[
                                                                                     cur_sensor_index],
                                                                                 timestamp)
                        if last_timestamp_return is not None:
                            last_timestamps[cur_sensor_name] = last_timestamp_return
                        if pcloud is not None:
                            output.write(sensor_gpulidar_topics[cur_sensor_index], pcloud, t=ros_timestamp)

                    for cur_sensor_index, cur_sensor_name in enumerate(sensor_uwb_names):
                        if cur_sensor_index == 0:  # only once
                            range_arrays = get_uwb_ros_message(client, vehicle_name, rospy, timestamp)
                            if len(range_arrays) != 0:
                                for range_array in range_arrays:
                                    output.write(sensor_uwb_topic, range_array, t=ros_timestamp)

                    for cur_sensor_index, cur_sensor_name in enumerate(sensor_wifi_names):
                        if cur_sensor_index == 0:  # only once
                            range_arrays = get_wifi_ros_message(client, vehicle_name, rospy, timestamp)
                            if len(range_arrays) != 0:
                                for range_array in range_arrays:
                                    output.write(sensor_wifi_topic, range_array, t=ros_timestamp)

                    if object_poses_all:
                        object_path = get_all_objects_ros_path_message(client, timestamp, first_message,
                                                                       object_poses_all_once,
                                                                       map_frame, object_poses_all_coordinates_local)
                        if object_path is not None:
                            output.write(object_poses_all_topic, object_path, t=ros_timestamp)

                    else:
                        for object_index, object_name in enumerate(object_poses_individual_names):
                            local_enable = object_poses_individual_coordinates_local[object_index]
                            object_pose, warning_issued_result = get_object_pose_ros_message(client, local_enable,
                                                                                             object_name,
                                                                                             warning_issued[
                                                                                                 object_name],
                                                                                             rospy, timestamp,
                                                                                             map_frame)
                            warning_issued[object_name] = warning_issued_result
                            if not warning_issued_result:
                                if object_pose is not None:
                                    output.write(object_poses_individual_topics[object_index], object_pose,
                                                 t=ros_timestamp)

                    pose_index += 1
                    first_message = False
                    time.sleep(1)

        output.write('/tf_static', saved_static_tf, first_timestamp)
        rospy.loginfo("Process completed. Writing all other messages to merged rosbag...")
        for topic, msg, t in route.read_messages():
            if topic != 'tf/static':
                output.write(topic, msg, t)
        output.close()
        rospy.loginfo("Merged rosbag with route and sensor data created!")
        if generate_gt_map:
            rospy.loginfo("Loading segmentation colormap...")
            colorMap = client.simGetSegmentationColorMap()
            rospy.loginfo("Loaded segmentation colormap.")

            currentObjectList = client.simListInstanceSegmentationObjects()
            rospy.loginfo("Generating list of all current objects...")
            with open('airsim_gt_map_' + datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + '.csv', 'w') as f:
                f.write("ObjectName,R,G,B\n")
                for index, item in enumerate(currentObjectList):
                    f.write("%s,%s\n" % (item, ','.join([str(x) for x in colorMap[index, :]])))
            rospy.loginfo("Generated ground truth map of " + str(len(currentObjectList)) + ' objects.')

    else:
        segmentation_warning_issued = False
        while not rospy.is_shutdown():

            timestamp = rospy.Time.now()

            try:
                cur_pose = client.simGetVehiclePose(vehicle_name)
            except msgpackrpc.error.RPCError:
                rospy.logerr("vehicle '" + vehicle_name + "' could not be found.")
                rospy.signal_shutdown('Vehicle not found.')
                sys.exit()

            cur_pos = cur_pose.position
            cur_orientation = cur_pose.orientation.inverse()

            pose_msg = PoseStamped()
            pose_msg.header.stamp = timestamp
            pose_msg.header.frame_id = map_frame
            pose_msg.header.seq = 1

            pose_msg.pose.position.x = cur_pos.x_val + pose_offset_x
            pose_msg.pose.position.y = -cur_pos.y_val + pose_offset_y
            pose_msg.pose.position.z = -cur_pos.z_val + pose_offset_z
            pose_msg.pose.orientation.w = cur_orientation.w_val
            pose_msg.pose.orientation.x = cur_orientation.x_val
            pose_msg.pose.orientation.y = cur_orientation.y_val
            pose_msg.pose.orientation.z = cur_orientation.z_val
            pose_publisher.publish(pose_msg)

            sim_odom = Odometry()
            if odometry_enable:
                sim_odom.header = pose_msg.header
                sim_odom.child_frame_id = odom_frame
                sim_odom.pose.pose = pose_msg.pose
                kinematics = client.simGetGroundTruthKinematics(vehicle_name)
                t_w_veh = tf.transformations.quaternion_matrix([cur_orientation.x_val, cur_orientation.y_val,
                                                                cur_orientation.z_val, cur_orientation.w_val])
                t_w_veh[:3, 3] = np.array([cur_pos.x_val, -cur_pos.y_val, -cur_pos.z_val])
                t_veh_w = t_invert(t_w_veh)

                local_linear = np.matmul(t_veh_w[:3, :3], np.array(
                    [kinematics.linear_velocity.x_val, - kinematics.linear_velocity.y_val,
                     - kinematics.linear_velocity.z_val]))
                local_angular = np.matmul(t_veh_w[:3, :3], np.array(
                    [kinematics.angular_velocity.x_val, - kinematics.angular_velocity.y_val,
                     - kinematics.angular_velocity.z_val]))
                sim_odom.twist.twist.linear.x = local_linear[0]
                sim_odom.twist.twist.linear.y = local_linear[1]
                sim_odom.twist.twist.linear.z = local_linear[2]
                sim_odom.twist.twist.angular.x = local_angular[0]
                sim_odom.twist.twist.angular.y = local_angular[1]
                sim_odom.twist.twist.angular.z = local_angular[2]
                odom_publisher.publish(sim_odom)

            if tf_odom_enable:
                tf_odom_pose = mat_2_tf(pose_2_mat(pose_msg.pose), timestamp, map_frame, odom_frame)
                tf_br.sendTransform(tf_odom_pose)

                tf_map_odom = TransformStamped()
                tf_map_odom.header.stamp = timestamp
                tf_map_odom.header.frame_id = odom_frame
                tf_map_odom.child_frame_id = vehicle_base_frame
                tf_map_odom.transform.rotation.w = 1
                tf_br.sendTransform(tf_map_odom)

            if sensor_imu_enable:
                imu_msg = get_imu_ros_message(client, sensor_imu_name, vehicle_name, timestamp, sensor_imu_frame)
                imu_publisher.publish(imu_msg)

            camera_responses = client.simGetImages(requests, vehicle_name)
            for cur_sensor_index, cur_sensor_name in enumerate(sensor_camera_names):
                response = camera_responses[response_locations[cur_sensor_name + '_scene']]
                if response.width == 0 and response.height == 0:
                    rospy.logwarn("Camera '" + cur_sensor_name + "' could not retrieve scene image.")
                else:
                    camera_msg = get_scene_camera_ros_message(response, timestamp,
                                                              sensor_camera_optical_frames[cur_sensor_index], cv_bridge,
                                                              sensor_camera_scene_quality[cur_sensor_index],
                                                              sensor_camera_toggle_scene_mono[cur_sensor_index])
                    image_publishers[cur_sensor_name + '_scene'].publish(camera_msg)

                if sensor_camera_toggle_segmentation[cur_sensor_index] == 1:
                    response = camera_responses[response_locations[cur_sensor_name + '_segmentation']]
                    if response.width == 0 and response.height == 0:
                        rospy.logwarn("Camera '" + cur_sensor_name + "' could not retrieve segmentation image.")
                    else:
                        camera_msg = get_segmentation_camera_ros_message(camera_msg, response)
                        image_publishers[cur_sensor_name + '_segmentation'].publish(camera_msg)
                    if not segmentation_warning_issued:
                        segmentation_warning_issued = True
                        rospy.logwarn("Instance segmentation is being used."
                                      " Do not forget to generate ground truth map!")
                if sensor_camera_toggle_depth[cur_sensor_index] == 1:
                    response = camera_responses[response_locations[cur_sensor_name + '_depth']]
                    if response.width == 0 and response.height == 0:
                        rospy.logwarn("Camera '" + cur_sensor_name + "' could not retrieve depth image.")
                    else:
                        camera_msg = get_depth_camera_ros_message(camera_msg, response)
                        image_publishers[cur_sensor_name + '_depth'].publish(camera_msg)
                if sensor_camera_toggle_annotation[cur_sensor_index] == 1:
                    response = camera_responses[response_locations[cur_sensor_name + '_annotation']]
                    if response.width == 0 and response.height == 0:
                        rospy.logwarn("Camera '" + cur_sensor_name + "' could not retrieve annotation image.")
                    else:
                        camera_msg = get_annotation_camera_ros_message(camera_msg, response)
                        image_publishers[cur_sensor_name + '_annotation'].publish(camera_msg)
                if sensor_camera_toggle_camera_info[cur_sensor_index] == 1:
                    if cur_sensor_index == 1:
                        first_sensor = True
                    else:
                        first_sensor = False
                    cam_info_msg = get_camera_info_ros_message(cameraInfo_objects[cur_sensor_name].fov,
                                                               sensor_camera_optical_frames[cur_sensor_index],
                                                               timestamp, response.width, response.height,
                                                               sensor_stereo_enable, baseline, first_sensor)
                    image_publishers[cur_sensor_name + '_cameraInfo'].publish(cam_info_msg)

            for cur_sensor_index, cur_sensor_name in enumerate(sensor_echo_names):
                pcloud, groundtruth, last_timestamp_return, pcloud_passive, groundtruth_passive = (
                    get_echo_ros_message(client, cur_sensor_name, vehicle_name, last_timestamps[cur_sensor_name],
                                         fields_echo, fields_echo_passive, sensor_echo_frames[cur_sensor_index],
                                         timestamp, sensor_echo_toggle_passive[cur_sensor_index]))
                if last_timestamp_return is not None:
                    last_timestamps[cur_sensor_name] = last_timestamp_return
                if pcloud is not None:
                    pointcloud_publishers[cur_sensor_name].publish(pcloud)
                if groundtruth is not None:
                    string_segmentation_publishers[cur_sensor_name].publish(groundtruth)
                if sensor_echo_toggle_passive[cur_sensor_index]:
                    if pcloud_passive is not None:
                        pointcloud_publishers[cur_sensor_name + "_passive"].publish(pcloud_passive)

                    if groundtruth_passive is not None:
                        string_segmentation_publishers[cur_sensor_name + "_passive"].publish(groundtruth_passive)

            for cur_sensor_index, cur_sensor_name in enumerate(sensor_lidar_names):
                seg_enable = sensor_lidar_toggle_segmentation[cur_sensor_index]
                cur_frame = sensor_lidar_frames[cur_sensor_index]
                pcloud, groundtruth, last_timestamp_return = get_lidar_ros_message(client, cur_sensor_name,
                                                                                   vehicle_name,
                                                                                   last_timestamps[cur_sensor_name],
                                                                                   cur_frame,
                                                                                   timestamp,
                                                                                   seg_enable)
                if last_timestamp_return is not None:
                    last_timestamps[cur_sensor_name] = last_timestamp_return
                if pcloud is not None:
                    pointcloud_publishers[cur_sensor_name].publish(pcloud)
                if sensor_lidar_toggle_segmentation[cur_sensor_index] == 1:
                    if groundtruth is not None:
                        string_segmentation_publishers[cur_sensor_name].publish(groundtruth)

            for cur_sensor_index, cur_sensor_name in enumerate(sensor_gpulidar_names):
                if not segmentation_warning_issued:
                    segmentation_warning_issued = True
                    rospy.logwarn("Instance segmentation is being used."
                                  " Do not forget to generate ground truth map!")
                pcloud, last_timestamp_return = get_gpulidar_ros_message(client, cur_sensor_name, vehicle_name,
                                                                         last_timestamps[cur_sensor_name],
                                                                         fields_lidar,
                                                                         sensor_gpulidar_frames[cur_sensor_index],
                                                                         timestamp)
                if last_timestamp_return is not None:
                    last_timestamps[cur_sensor_name] = last_timestamp_return
                if pcloud is not None:
                    pointcloud_publishers[cur_sensor_name].publish(pcloud)

            for cur_sensor_index, cur_sensor_name in enumerate(sensor_uwb_names):
                if cur_sensor_index == 0:  # only once
                    range_arrays = get_uwb_ros_message(client, vehicle_name, rospy, timestamp)
                    if len(range_arrays) != 0:
                        for range_array in range_arrays:
                            uwb_range_array_publisher.publish(range_array)

            for cur_sensor_index, cur_sensor_name in enumerate(sensor_wifi_names):
                if cur_sensor_index == 0:  # only once
                    range_arrays = get_wifi_ros_message(client, vehicle_name, rospy, timestamp)
                    if len(range_arrays) != 0:
                        for range_array in range_arrays:
                            wifi_range_array_publisher.publish(range_array)

            if object_poses_all:
                object_path = get_all_objects_ros_path_message(client, timestamp, first_message, object_poses_all_once,
                                                               map_frame, object_poses_all_coordinates_local)
                if object_path is not None:
                    object_path_publisher.publish(object_path)

            else:
                for object_index, object_name in enumerate(object_poses_individual_names):
                    local_enable = object_poses_individual_coordinates_local[object_index]
                    object_pose, warning_issued_result = get_object_pose_ros_message(client, local_enable, object_name,
                                                                                     warning_issued[object_name],
                                                                                     rospy, timestamp, map_frame)
                    warning_issued[object_name] = warning_issued_result
                    if not warning_issued_result:
                        if object_pose is not None:
                            objectpose_publishers[object_name].publish(object_pose)

            if carcontrol_enable == 1:
                desired_speed, v, error, desired_rotation_return = get_car_control_ros_message(client,
                                                                                               queue_input_command,
                                                                                               speed_pid,
                                                                                               sim_odom,
                                                                                               vehicle_name,
                                                                                               desired_rotation)
                desired_rotation = desired_rotation_return
                request_speed_publisher.publish(desired_speed)
                real_speed_publisher.publish(v)
                error_speed_publisher.publish(error)

            first_message = False
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('airsim_play_route_record_sensors', anonymous=True)

        ros_rate = rospy.get_param('~rate', 10)
        ip = rospy.get_param('~ip', "localhost")
        port = rospy.get_param('~port', 41451)

        tf_sensors_enable = rospy.get_param('~tf_sensors_enable', 0)

        toggle_drone = rospy.get_param('~toggle_drone', 0)

        use_route = rospy.get_param('~use_route', 0)
        route_rosbag = rospy.get_param('~route_rosbag', "./airsim_route_only.bag")
        merged_rosbag = rospy.get_param('~merged_rosbag', "./airsim_sensor_data.bag")
        generate_gt_map = rospy.get_param('~generate_gt_map', 0)

        vehicle_name = rospy.get_param('~vehicle_name', "airsimvehicle")
        vehicle_base_frame = rospy.get_param('~vehicle_base_frame', "base_link")

        pose_topic = rospy.get_param('~pose_topic', 0)
        map_frame = rospy.get_param('~map_frame', "map")

        carcontrol_enable = rospy.get_param('~carcontrol_enable', 0)
        carcontrol_topic = rospy.get_param('~carcontrol_topic', "/airsim/carcontrol")

        odometry_enable = rospy.get_param('~odometry_enable', 0)
        odometry_topic = rospy.get_param('~odometry_topic', "airsim/odom")
        odom_frame = rospy.get_param('~odom_frame', "odom")
        tf_odom_enable = rospy.get_param('~tf_odom_enable', 0)

        pose_offset_x = rospy.get_param('~pose_offset_x', 0)
        pose_offset_y = rospy.get_param('~pose_offset_y', 0)
        pose_offset_z = rospy.get_param('~pose_offset_z', 0)

        sensor_imu_enable = rospy.get_param('~sensor_imu_enable', 0)
        sensor_imu_name = rospy.get_param('~sensor_imu_name', "imu")
        sensor_imu_topic = rospy.get_param('~sensor_imu_topic', "airsim/imu")
        sensor_imu_frame = rospy.get_param('~sensor_imu_frame', "base_imu")

        sensor_echo_names = rospy.get_param('~sensor_echo_names', [])
        sensor_echo_topics = rospy.get_param('~sensor_echo_topics', "airsim/echo1/active/pointcloud")
        sensor_echo_segmentation_topics = rospy.get_param('~sensor_echo_segmentation_topics',
                                                          "airsim/echo1/active/segmentation")
        sensor_echo_frames = rospy.get_param('~sensor_echo_frames', "base_echo1")
        sensor_echo_toggle_passive = rospy.get_param('~sensor_echo_toggle_passive', 1)
        sensor_echo_passive_topics = rospy.get_param('~sensor_echo_passive_topics', "airsim/echo1/passive/pointcloud")
        sensor_echo_passive_segmentation_topics = rospy.get_param('~sensor_echo_passive_segmentation_topics',
                                                                  "airsim/echo1/passive/segmentation")

        sensor_lidar_names = rospy.get_param('~sensor_lidar_names', [])
        sensor_lidar_toggle_groundtruth = rospy.get_param('~sensor_lidar_toggle_groundtruth', 1)
        sensor_lidar_topics = rospy.get_param('~sensor_lidar_topics', "airsim/lidar1/pointcloud")
        sensor_lidar_segmentation_topics = rospy.get_param('~sensor_lidar_segmentation_topics',
                                                           "airsim/lidar1/segmentation")
        sensor_lidar_frames = rospy.get_param('~sensor_lidar_frames', "base_lidar1")

        sensor_gpulidar_names = rospy.get_param('~sensor_gpulidar_names', [])
        sensor_gpulidar_topics = rospy.get_param('~sensor_gpulidar_topics', "airsim/lidar2/pointcloud")
        sensor_gpulidar_frames = rospy.get_param('~sensor_gpulidar_frames', "base_lidar2")

        sensor_uwb_names = rospy.get_param('~sensor_uwb_names', [])
        sensor_uwb_topic = rospy.get_param('~sensor_uwb_topic', "airsim/uwb1/ranges")
        sensor_uwb_frames = rospy.get_param('~sensor_uwb_frames', "base_uwb1")

        sensor_wifi_names = rospy.get_param('~sensor_wifi_names', [])
        sensor_wifi_topic = rospy.get_param('~sensor_wifi_topic', "airsim/wifi1/ranges")
        sensor_wifi_frames = rospy.get_param('~sensor_wifi_frames', "base_wifi1")

        sensor_camera_names = rospy.get_param('~sensor_camera_names', [])
        sensor_camera_toggle_scene_mono = rospy.get_param('~sensor_camera_toggle_scene_mono', 0)
        sensor_camera_scene_quality = rospy.get_param('~sensor_camera_scene_quality', 0)
        sensor_camera_toggle_segmentation = rospy.get_param('~sensor_camera_toggle_segmentation', 1)
        sensor_camera_toggle_depth = rospy.get_param('~sensor_camera_toggle_depth', 1)
        sensor_camera_toggle_annotation = rospy.get_param('~sensor_camera_toggle_annotation', 0)
        sensor_camera_annotation_layers = rospy.get_param('~sensor_camera_annotation_layers',
                                                          ["TextureTestDirect", "GreyscaleTest"])
        sensor_camera_scene_topics = rospy.get_param('~sensor_camera_scene_topics',
                                                     ["airsim/leftcamera/rgb/image", "airsim/rightcamera/rgb/image"])
        sensor_camera_segmentation_topics = rospy.get_param('~sensor_camera_segmentation_topics',
                                                            ["airsim/leftcamera/segmentation/image",
                                                             "airsim/rightcamera/segmentation/image"])
        sensor_camera_depth_topics = rospy.get_param('~sensor_camera_depth_topics',
                                                     ["airsim/leftcamera/depth/image",
                                                      "airsim/rightcamera/depth/image"])
        sensor_camera_annotation_topics = rospy.get_param('~sensor_camera_annotation_topics',
                                                          ["airsim/frontcamera/annotation/TextureTestDirect/image",
                                                           "airsim/backcamera/annotation/GreyscaleTest/image"])
        sensor_camera_frames = rospy.get_param('~sensor_camera_frames', ["base_leftcamera", "base_rightcamera"])
        sensor_camera_optical_frames = rospy.get_param('~sensor_camera_optical_frames',
                                                       ["leftcamera_optical_link", "rightcamera_optical_link"])
        sensor_camera_toggle_camera_info = rospy.get_param('~sensor_camera_toggle_camera_info', [1, 1])
        sensor_camera_info_topics = rospy.get_param('~sensor_camera_info_topics',
                                                    ["airsim/leftcamera/camera_info",
                                                     "airsim/rightcamera/camera_info"])
        sensor_stereo_enable = rospy.get_param('~sensor_stereo_enable', 1)

        object_poses_all = rospy.get_param('~object_poses_all', 0)
        object_poses_all_coordinates_local = rospy.get_param('~object_poses_all_coordinates_local', 1)
        object_poses_all_once = rospy.get_param('~object_poses_all_once', 0)
        object_poses_all_topic = rospy.get_param('~object_poses_all_topic', "airsim/objects/poses")

        object_poses_individual_names = rospy.get_param('~object_poses_individual_names', [])
        object_poses_individual_coordinates_local = rospy.get_param('~object_poses_individual_coordinates_local',
                                                                    1)
        object_poses_individual_topics = rospy.get_param('~object_poses_individual_topics',
                                                         "airsim/object/object_name/pose")

        rospy.loginfo("Connecting to AirSim...")
        if toggle_drone:
            client = airsim.MultirotorClient(ip=ip, port=port)
        else:
            client = airsim.CarClient(ip=ip, port=port)
        try:
            client.confirmConnection()
        except msgpackrpc.error.TimeoutError:
            rospy.logerr("Could not connect to AirSim.")
            rospy.signal_shutdown('no connection to airsim.')
            sys.exit()
        rospy.loginfo("Connected to AirSim!")
        tf_static = None
        if tf_sensors_enable:
            rospy.loginfo("Starting static transforms...")

            if use_route:
                tf_static = tf2_msgs.msg.TFMessage()
            else:
                broadcaster = tf2_ros.StaticTransformBroadcaster()
                transform_list = []
            if sensor_imu_enable:
                try:
                    echo_data = client.getImuData(sensor_imu_name, vehicle_name)
                except msgpackrpc.error.RPCError:
                    rospy.logerr("IMU sensor '" + sensor_imu_name + "' could not be found.")
                    rospy.signal_shutdown('Sensor not found.')
                    sys.exit()
                static_transform = TransformStamped()
                static_transform.header.stamp = rospy.Time.now()
                static_transform.header.frame_id = vehicle_base_frame
                static_transform.child_frame_id = sensor_imu_frame
                static_transform.transform.translation.x = 0
                static_transform.transform.translation.y = 0
                static_transform.transform.translation.z = 0
                static_transform.transform.rotation.x = 0
                static_transform.transform.rotation.y = 0
                static_transform.transform.rotation.z = 0
                static_transform.transform.rotation.w = 1
                if use_route:
                    tf_static.transforms.append(static_transform)
                else:
                    transform_list.append(static_transform)
                time.sleep(0.1)
                rospy.loginfo("Started static transform for IMU sensor with ID " + sensor_imu_name + ".")

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
                if use_route:
                    tf_static.transforms.append(static_transform)
                else:
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
                if use_route:
                    tf_static.transforms.append(static_transform)
                else:
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
                if use_route:
                    tf_static.transforms.append(static_transform)
                else:
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

                if len(uwb_data) == 2:
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
                    if use_route:
                        tf_static.transforms.append(static_transform)
                    else:
                        transform_list.append(static_transform)
                    time.sleep(0.1)
                    rospy.loginfo("Started static transform for UWB sensor with ID " + sensor_name + ".")

            left_position = None
            right_position = None
            for sensor_index, sensor_name in enumerate(sensor_camera_names):
                try:
                    camera_data = client.simGetCameraInfo(sensor_name, vehicle_name=vehicle_name)
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
                if use_route:
                    tf_static.transforms.append(static_transform)
                else:
                    transform_list.append(static_transform)

                # optical frame
                static_transform = TransformStamped()
                static_transform.header.stamp = rospy.Time.now()
                static_transform.header.frame_id = sensor_camera_frames[sensor_index]
                static_transform.child_frame_id = sensor_camera_optical_frames[sensor_index]
                static_transform.transform.translation.x = 0
                static_transform.transform.translation.y = 0
                static_transform.transform.translation.z = 0
                q_rot = tf.transformations.quaternion_from_euler(-math.pi / 2, 0, -math.pi / 2)
                static_transform.transform.rotation.x = q_rot[0]
                static_transform.transform.rotation.y = q_rot[1]
                static_transform.transform.rotation.z = q_rot[2]
                static_transform.transform.rotation.w = q_rot[3]
                if use_route:
                    tf_static.transforms.append(static_transform)
                else:
                    transform_list.append(static_transform)

                time.sleep(0.1)
                rospy.loginfo("Started static transform for camera sensor with ID " + sensor_name + ".")
                if sensor_stereo_enable == 1 and sensor_index == 0:
                    left_position = [pose.position.x_val, -pose.position.y_val, -pose.position.z_val]
                elif sensor_stereo_enable == 1 and sensor_index == 1:
                    right_position = [pose.position.x_val, -pose.position.y_val, -pose.position.z_val]
            if left_position is not None and right_position is not None:
                baseline = math.sqrt((left_position[0] - right_position[0]) ** 2 + (left_position[1]
                                                                                    - right_position[1]) ** 2
                                     + (left_position[2] - right_position[2]) ** 2)
            else:
                baseline = 0
            if not use_route:
                broadcaster.sendTransform(transform_list)
        else:
            baseline = 0

        airsim_publish(client, use_route, route_rosbag, merged_rosbag, generate_gt_map, tf_static,
                       vehicle_name, pose_topic, map_frame, odom_frame, tf_odom_enable,
                       pose_offset_x, pose_offset_y, pose_offset_z,
                       carcontrol_enable, carcontrol_topic,
                       odometry_enable, odometry_topic, sensor_imu_enable,
                       sensor_imu_name, sensor_imu_topic, sensor_imu_frame, sensor_echo_names,
                       sensor_echo_topics, sensor_echo_segmentation_topics, sensor_echo_frames,
                       sensor_echo_toggle_passive, sensor_echo_passive_topics, sensor_echo_passive_segmentation_topics,
                       sensor_lidar_names, sensor_lidar_toggle_groundtruth,
                       sensor_lidar_topics, sensor_lidar_segmentation_topics, sensor_lidar_frames,
                       sensor_gpulidar_names, sensor_gpulidar_topics,
                       sensor_gpulidar_frames, sensor_camera_names, sensor_camera_toggle_scene_mono,
                       sensor_camera_scene_quality, sensor_camera_toggle_segmentation,
                       sensor_camera_toggle_depth, sensor_camera_toggle_annotation, sensor_camera_annotation_layers,
                       sensor_camera_scene_topics, sensor_camera_segmentation_topics, sensor_camera_depth_topics,
                       sensor_camera_annotation_topics, sensor_camera_optical_frames, sensor_camera_toggle_camera_info,
                       sensor_camera_info_topics, sensor_stereo_enable, baseline,
                       object_poses_all, object_poses_all_coordinates_local, object_poses_all_once,
                       object_poses_all_topic, object_poses_individual_names, object_poses_individual_coordinates_local,
                       object_poses_individual_topics, sensor_uwb_names, sensor_uwb_topic, sensor_wifi_names,
                       sensor_wifi_topic)

    except rospy.ROSInterruptException:
        pass
