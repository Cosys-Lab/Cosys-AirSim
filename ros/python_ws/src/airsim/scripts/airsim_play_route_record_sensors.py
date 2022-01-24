#!/usr/bin/env python

import setup_path
import airsimpy
import rospy
import time
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, TransformStamped
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from airsim.msg import StringArray
from sensor_msgs.msg import PointCloud2, PointField
import rosbag
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

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


def airsim_play_route_record_sensors(client, vehicle_name, pose_topic, pose_frame, sensor_echo_names,
                                     sensor_echo_external, sensor_echo_topics, sensor_echo_frames, sensor_lidar_names,
                                     sensor_lidar_toggle_external, sensor_lidar_toggle_segmentation,
                                     sensor_lidar_topics, sensor_lidar_segmentation_topics, sensor_lidar_frames,
                                     sensor_gpulidar_names, sensor_gpulidar_external, sensor_gpulidar_topics,
                                     sensor_gpulidar_frames, sensor_camera_names, sensor_camera_toggle_scene_mono,
                                     sensor_camera_scene_quality, sensor_camera_toggle_segmentation,
                                     sensor_camera_toggle_depth, sensor_camera_scene_topics,
                                     sensor_camera_segmentation_topics, sensor_camera_depth_topics,
                                     sensor_camera_frames, object_names, object_topics,
                                     route_rosbag, merged_rosbag):

    rospy.loginfo("Reading route...")
    route = rosbag.Bag(route_rosbag)
    output = rosbag.Bag(merged_rosbag, 'w')
    rospy.loginfo("Route retrieved!")

    last_timestamps = {}

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

    print("Starting...")
    rospy.logwarn("Ensure focus is on the screen of AirSim simulator to allow auto configuration!")
    rospy.logdebug(str(route.get_type_and_topic_info()))

    for _, msg, t in route.read_messages(topics=pose_topic):
        if rospy.is_shutdown():
            break

        ros_timestamp = t
        timestamp = msg.header.stamp

        position = airsimpy.Vector3r(msg.pose.position.x, -msg.pose.position.y, -msg.pose.position.z)
        orientation = airsimpy.Quaternionr(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                                           msg.pose.orientation.w).inverse()
        orientation = airsimpy.Quaternionr(float(orientation.x_val), float(orientation.y_val),
                                           float(orientation.z_val), float(orientation.w_val))
        client.simSetVehiclePose(airsimpy.Pose(position, orientation), True, vehicle_name)
        rospy.logdebug("Setting vehicle pose at {}".format(str(t)))

        camera_responses = client.simGetImages(requests)

        for sensor_index, sensor_name in enumerate(sensor_camera_names):
            response = camera_responses[response_locations[sensor_name + '_scene']]
            rgb_matrix = np.fromstring(get_image_bytes(response, "Scene"), dtype=np.uint8).reshape(response.height,
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
            camera_msg.header.frame_id = sensor_camera_frames[sensor_index]
            output.write(sensor_camera_scene_topics[sensor_index], camera_msg, t=ros_timestamp)

            if sensor_camera_toggle_segmentation[sensor_index] is 1:
                response = camera_responses[response_locations[sensor_name + '_segmentation']]
                rgb_string = get_image_bytes(response, "Segmentation")
                camera_msg.step = response.width * 3
                camera_msg.data = rgb_string
                output.write(sensor_camera_segmentation_topics[sensor_index], camera_msg, t=ros_timestamp)
            if sensor_camera_toggle_depth[sensor_index] is 1:
                response = camera_responses[response_locations[sensor_name + '_depth']]
                rgb_string = get_image_bytes(response, "DepthPlanner")
                camera_msg.encoding = "32FC1"
                camera_msg.step = response.width * 4
                camera_msg.data = rgb_string
                output.write(sensor_camera_depth_topics[sensor_index], camera_msg, t=ros_timestamp)

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
                    output.write(sensor_echo_topics[sensor_index], pcloud, t=ros_timestamp)

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

                    output.write(sensor_lidar_topics[sensor_index], pcloud, t=ros_timestamp)

                    if sensor_lidar_toggle_segmentation[sensor_index] is 1:
                        labels = np.array(lidar_data.groundtruth, dtype=np.dtype('U'))
                        groundtruth = StringArray()
                        groundtruth.data = labels.tolist()
                        groundtruth.header.frame_id = sensor_lidar_frames[sensor_index]
                        groundtruth.header.stamp = timestamp
                        output.write(sensor_lidar_segmentation_topics[sensor_index], groundtruth, t=ros_timestamp)

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

                    output.write(sensor_gpulidar_topics[sensor_index], pcloud, t=ros_timestamp)

        for object_index, object_name in enumerate(object_names):
            pose = client.simGetObjectPose(object_name, False)
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

            output.write(object_topics[sensor_index], object_pose, t=ros_timestamp)

    print("Process completed. Writing all messages to merged rosbag...")
    for topic, msg, t in route.read_messages():
        output.write(topic, msg, t)
    output.close()
    print("Merged rosbag with route and sensor data created!")


if __name__ == '__main__':
    try:
        rospy.init_node('airsim_play_route_record_sensors', anonymous=True)

        ip = rospy.get_param('~ip', "")

        toggle_drone = rospy.get_param('~toggle_drone', 0)

        vehicle_name = rospy.get_param('~vehicle_name', 'airsimvehicle')
        vehicle_base_frame = rospy.get_param('~vehicle_base_frame', 'base_link')

        pose_topic = rospy.get_param('~pose_topic', "airsim/gtpose")
        pose_frame = rospy.get_param('~pose_frame', "world")

        route_rosbag = rospy.get_param('~route_rosbag', "airsim_route_only.bag")
        merged_rosbag = rospy.get_param('~merged_rosbag', "airsim_route_sensors.bag")
        
        sensor_echo_names = rospy.get_param('~sensor_echo_names', "True")
        sensor_echo_external = rospy.get_param('~sensor_echo_external', "True")
        sensor_echo_topics = rospy.get_param('~sensor_echo_topics', "True")
        sensor_echo_frames = rospy.get_param('~sensor_echo_frames', "True")

        sensor_lidar_names = rospy.get_param('~sensor_lidar_names', "True")
        sensor_lidar_toggle_external = rospy.get_param('~sensor_lidar_toggle_external', "True")
        sensor_lidar_toggle_groundtruth = rospy.get_param('~sensor_lidar_toggle_groundtruth', "True")
        sensor_lidar_topics = rospy.get_param('~sensor_lidar_topics', "True")
        sensor_lidar_segmentation_topics = rospy.get_param('~sensor_lidar_segmentation_topics', "True")
        sensor_lidar_frames = rospy.get_param('~sensor_lidar_frames', "True")

        sensor_gpulidar_names = rospy.get_param('~sensor_gpulidar_names', "True")
        sensor_gpulidar_external = rospy.get_param('~sensor_gpulidar_external', "True")
        sensor_gpulidar_topics = rospy.get_param('~sensor_gpulidar_topics', "True")
        sensor_gpulidar_frames = rospy.get_param('~sensor_gpulidar_frames', "True")

        sensor_camera_names = rospy.get_param('~sensor_camera_names', "True")
        sensor_camera_toggle_scene_mono = rospy.get_param('~sensor_camera_toggle_scene_mono', "True")
        sensor_camera_scene_quality = rospy.get_param('~sensor_camera_scene_quality', "True")
        sensor_camera_toggle_segmentation = rospy.get_param('~sensor_camera_toggle_segmentation', "True")
        sensor_camera_toggle_depth = rospy.get_param('~sensor_camera_toggle_depth', "True")
        sensor_camera_scene_topics = rospy.get_param('~sensor_camera_scene_topics', "True")
        sensor_camera_segmentation_topics = rospy.get_param('~sensor_camera_segmentation_topics', "True")
        sensor_camera_depth_topics = rospy.get_param('~sensor_camera_depth_topics', "True")
        sensor_camera_frames = rospy.get_param('~sensor_camera_frames', "True")

        object_names = rospy.get_param('~object_names', "True")
        object_topics = rospy.get_param('~object_topics', "True")

        print("Connecting to AirSim...")
        if toggle_drone:
            client = airsimpy.MultirotorClient(ip)
        else:
            client = airsimpy.CarClient(ip)
        client.confirmConnection(rospy.get_name())
        print("Connected to AirSim!")

        rospy.loginfo("Starting static transforms...")
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        transform_list = []

        for sensor_index, sensor_name in enumerate(sensor_echo_names):
            echo_data = client.getEchoData(sensor_name, vehicle_name)
            pose = echo_data.pose
            static_transform = TransformStamped()
            static_transform.header.stamp = rospy.Time.now()
            static_transform.header.frame_id = vehicle_base_frame
            static_transform.child_frame_id = sensor_echo_frames[sensor_index]
            static_transform.transform.translation.x = pose.position.x_val
            static_transform.transform.translation.y = -pose.position.y_val
            static_transform.transform.translation.z = -pose.position.z_val
            static_transform.transform.rotation.x = pose.orientation.x_val
            static_transform.transform.rotation.y = pose.orientation.y_val
            static_transform.transform.rotation.z = pose.orientation.z_val
            static_transform.transform.rotation.w = pose.orientation.w_val
            transform_list.append(static_transform)
            time.sleep(0.1)
            rospy.loginfo("Started static transform for echo sensor with ID " + sensor_name + ".")

        for sensor_index, sensor_name in enumerate(sensor_lidar_names):
            lidar_data = client.getLidarData(sensor_name, vehicle_name)
            pose = lidar_data.pose
            static_transform = TransformStamped()
            static_transform.header.stamp = rospy.Time.now()
            static_transform.header.frame_id = vehicle_base_frame
            static_transform.child_frame_id = sensor_lidar_frames[sensor_index]
            static_transform.transform.translation.x = pose.position.x_val
            static_transform.transform.translation.y = -pose.position.y_val
            static_transform.transform.translation.z = -pose.position.z_val
            static_transform.transform.rotation.x = pose.orientation.x_val
            static_transform.transform.rotation.y = pose.orientation.y_val
            static_transform.transform.rotation.z = pose.orientation.z_val
            static_transform.transform.rotation.w = pose.orientation.w_val
            transform_list.append(static_transform)
            time.sleep(0.1)
            rospy.loginfo("Started static transform for LiDAR sensor with ID " + sensor_name + ".")

        for sensor_index, sensor_name in enumerate(sensor_gpulidar_names):
            lidar_data = client.getGPULidarData(sensor_name, vehicle_name)
            pose = lidar_data.pose
            static_transform = TransformStamped()
            static_transform.header.stamp = rospy.Time.now()
            static_transform.header.frame_id = vehicle_base_frame
            static_transform.child_frame_id = sensor_gpulidar_frames[sensor_index]
            static_transform.transform.translation.x = pose.position.x_val
            static_transform.transform.translation.y = -pose.position.y_val
            static_transform.transform.translation.z = -pose.position.z_val
            static_transform.transform.rotation.x = pose.orientation.x_val
            static_transform.transform.rotation.y = pose.orientation.y_val
            static_transform.transform.rotation.z = pose.orientation.z_val
            static_transform.transform.rotation.w = pose.orientation.w_val
            transform_list.append(static_transform)
            time.sleep(0.1)
            rospy.loginfo("Started static transform for GPU-LiDAR sensor with ID " + sensor_name + ".")

        for sensor_index, sensor_name in enumerate(sensor_camera_names):
            camera_data = client.getCameraInfo(sensor_name)
            pose = camera_data.pose
            static_transform = TransformStamped()
            static_transform.header.stamp = rospy.Time.now()
            static_transform.header.frame_id = vehicle_base_frame
            static_transform.child_frame_id = sensor_gpulidar_frames[sensor_index]
            static_transform.transform.translation.x = pose.position.x_val
            static_transform.transform.translation.y = -pose.position.y_val
            static_transform.transform.translation.z = -pose.position.z_val
            static_transform.transform.rotation.x = pose.orientation.x_val
            static_transform.transform.rotation.y = pose.orientation.y_val
            static_transform.transform.rotation.z = pose.orientation.z_val
            static_transform.transform.rotation.w = pose.orientation.w_val
            transform_list.append(static_transform)
            time.sleep(0.1)
            rospy.loginfo("Started static transform for camera sensor with ID " + sensor_name + ".")

        broadcaster.sendTransform(transform_list)

        airsim_play_route_record_sensors(client, vehicle_name, pose_topic, pose_frame,
                                         sensor_echo_names, sensor_echo_external,
                                         sensor_echo_topics, sensor_echo_frames, sensor_lidar_names,
                                         sensor_lidar_toggle_external, sensor_lidar_toggle_groundtruth,
                                         sensor_lidar_topics, sensor_lidar_segmentation_topics, sensor_lidar_frames,
                                         sensor_gpulidar_names, sensor_gpulidar_external, sensor_gpulidar_topics,
                                         sensor_gpulidar_frames, sensor_camera_names, sensor_camera_toggle_scene_mono,
                                         sensor_camera_scene_quality, sensor_camera_toggle_segmentation,
                                         sensor_camera_toggle_depth, sensor_camera_scene_topics,
                                         sensor_camera_segmentation_topics, sensor_camera_depth_topics,
                                         sensor_camera_frames, object_names, object_topics, route_rosbag, merged_rosbag)

    except rospy.ROSInterruptException:
        pass
