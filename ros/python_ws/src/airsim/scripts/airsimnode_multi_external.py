#!/usr/bin/env python

import time
import setup_path
import airsimpy
import rospy
from std_msgs.msg import String, Header
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf2_ros
import geometry_msgs

print("Connecting...")
client = airsimpy.CarClient("172.30.128.1")
client.confirmConnection(rospy.get_name())

print("Connected to AirSim!")


def get_valid_ros_topic_nameid(sensor_name):
    sensor_name_fixed = sensor_name.replace("-","_")
    sensor_name_fixed = sensor_name_fixed.replace("__", "_")
    sensor_name_fixed = sensor_name_fixed.replace("//", "/")
    if sensor_name_fixed[0].isdigit():
        sensor_name_fixed = "a" + sensor_name_fixed
    return sensor_name_fixed


def airsim_echos_pub(ros_rate, vehicle_name, echo_topic_prefix, echo_topic_suffix, lidar_topic_prefix,
                         lidar_topic_suffix, object_topic_prefix, object_topic_suffix,
                         echo_frame_prefix, lidar_frame_prefix, base_frame,
                         echo_names, lidar_names, cpulidar_names, object_names):

    rospy.Rate(ros_rate)

    object_pose_publishers = {}
    pointcloud_publishers = {}
    last_timestamps = {}
    for sensor_name in echo_names:
        pointcloud_publishers[sensor_name] = rospy.Publisher(echo_topic_prefix + "/"
                                                             + str(get_valid_ros_topic_nameid(sensor_name))
                                                             + "/" + echo_topic_suffix, PointCloud2, queue_size=2)
        last_timestamps[sensor_name] = None
    for sensor_name in lidar_names:
        pointcloud_publishers[sensor_name] = rospy.Publisher(lidar_topic_prefix + "/"
                                                             + str(get_valid_ros_topic_nameid(sensor_name))
                                                             + "/" + lidar_topic_suffix, PointCloud2, queue_size=2)
        last_timestamps[sensor_name] = None
    for sensor_name in cpulidar_names:
        pointcloud_publishers[sensor_name] = rospy.Publisher(lidar_topic_prefix + "/"
                                                             + str(get_valid_ros_topic_nameid(sensor_name))
                                                             + "/" + lidar_topic_suffix, PointCloud2, queue_size=2)
        last_timestamps[sensor_name] = None
    for object_name in object_names:
            object_pose_publishers[object_name] = rospy.Publisher(object_topic_prefix + "/"
                                                                  + str(get_valid_ros_topic_nameid(object_name))
                                                                  + "/" + object_topic_suffix, PoseStamped,
                                                                  queue_size=1)

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

    rospy.loginfo("Started publishers...")

    while not rospy.is_shutdown():

        timestamp = rospy.Time.now()

        for object_name in object_names:
            pose = client.simGetObjectPose(object_name, False)
            pos = pose.position
            orientation = pose.orientation.inverse()

            simPose = PoseStamped()
            simPose.pose.position.x = pos.x_val
            simPose.pose.position.y = -pos.y_val
            simPose.pose.position.z = pos.z_val
            simPose.pose.orientation.w = orientation.w_val
            simPose.pose.orientation.x = orientation.x_val
            simPose.pose.orientation.y = orientation.y_val
            simPose.pose.orientation.z = orientation.z_val
            simPose.header.stamp = timestamp
            simPose.header.seq = 1
            simPose.header.frame_id = base_frame

            object_pose_publishers[object_name].publish(simPose)

        for sensor_name in lidar_names:
            lidar_data = client.getGPULidarData(sensor_name, vehicle_name)

            if lidar_data.time_stamp != last_timestamps[sensor_name]:
                if len(lidar_data.point_cloud) < 5:
                    last_timestamps[sensor_name] = lidar_data.time_stamp
                else:
                    last_timestamps[sensor_name] = lidar_data.time_stamp

                    pcloud = PointCloud2()

                    points = np.array(lidar_data.point_cloud, dtype=np.dtype('f4'))
                    points = np.reshape(points, (int(points.shape[0] / 5), 5))
                    pcloud.header.frame_id = lidar_frame_prefix + "_" + str(get_valid_ros_topic_nameid(sensor_name))
                    pcloud.header.stamp = timestamp

                    pcloud = pc2.create_cloud(pcloud.header, fields_lidar, points.tolist())
                    pointcloud_publishers[sensor_name].publish(pcloud)

        for sensor_name in cpulidar_names:
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
                    pcloud.header.frame_id = lidar_frame_prefix + "_" + str(get_valid_ros_topic_nameid(sensor_name))
                    pcloud.header.stamp = timestamp
                    pcloud = pc2.create_cloud_xyz32(pcloud.header, cloud)

                    pointcloud_publishers[sensor_name].publish(pcloud)

        for sensor_name in echo_names:
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
                    header.frame_id = echo_frame_prefix + "_" + str(get_valid_ros_topic_nameid(sensor_name))

                    pcloud = pc2.create_cloud(header, fields_echo, points_list)
                    pcloud.header.stamp = timestamp
                    pointcloud_publishers[sensor_name].publish(pcloud)


if __name__ == '__main__':
    try:
        rospy.init_node('airsimnode_multi_external_publisher', anonymous=True)
        ros_rate = rospy.get_param('~ros_rate', 100)

        echo_topic_prefix = rospy.get_param('~echo_topic_prefix', "echo")
        echo_topic_suffix = rospy.get_param('~echo_topic_suffix', "pointcloud")
        lidar_topic_prefix = rospy.get_param('~lidar_topic_prefix', "lidar")
        lidar_topic_suffix = rospy.get_param('~lidar_topic_suffix', "pointcloud")
        object_topic_prefix = rospy.get_param('~object_topic_prefix', "object")
        object_topic_suffix = rospy.get_param('~object_topic_suffix', "pose")
        vehicle_name = rospy.get_param('~vehicle_name', 'airsimvehicle')
        echo_frame_prefix = rospy.get_param('~echo_frame_prefix', "base_echo")
        lidar_frame_prefix = rospy.get_param('~lidar_frame_prefix', "base_lidar")
        base_frame = rospy.get_param('~base_frame', "world")
        echo_names = rospy.get_param('~echo_names', "True")
        lidar_names = rospy.get_param('~lidar_names', "True")
        cpulidar_names = rospy.get_param('~cpulidar_names', "True")
        object_names = rospy.get_param('~object_names', "True")

        rospy.loginfo("Starting static transforms...")
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        transformList = []

        for sensor_name in echo_names:
            echo_data = client.getEchoData(sensor_name, vehicle_name)
            pose = echo_data.pose
            static_transformStamped = geometry_msgs.msg.TransformStamped()
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = base_frame
            static_transformStamped.child_frame_id = str(echo_frame_prefix) + "_" + str(get_valid_ros_topic_nameid(sensor_name))
            static_transformStamped.transform.translation.x = pose.position.x_val
            static_transformStamped.transform.translation.y = -pose.position.y_val
            static_transformStamped.transform.translation.z = pose.position.z_val
            static_transformStamped.transform.rotation.x = pose.orientation.x_val
            static_transformStamped.transform.rotation.y = pose.orientation.y_val
            static_transformStamped.transform.rotation.z = pose.orientation.z_val
            static_transformStamped.transform.rotation.w = pose.orientation.w_val
            transformList.append(static_transformStamped)
            time.sleep(1)
            rospy.loginfo("Started static transform for echo sensor with ID " + sensor_name + ".")

        for sensor_name in lidar_names:
            lidar_data = client.getGPULidarData(sensor_name, vehicle_name)
            pose = lidar_data.pose
            static_transformStamped = geometry_msgs.msg.TransformStamped()
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = base_frame
            static_transformStamped.child_frame_id = str(lidar_frame_prefix) + "_" + str(get_valid_ros_topic_nameid(sensor_name))
            static_transformStamped.transform.translation.x = pose.position.x_val
            static_transformStamped.transform.translation.y = -pose.position.y_val
            static_transformStamped.transform.translation.z = pose.position.z_val
            static_transformStamped.transform.rotation.x = pose.orientation.x_val
            static_transformStamped.transform.rotation.y = pose.orientation.y_val
            static_transformStamped.transform.rotation.z = pose.orientation.z_val
            static_transformStamped.transform.rotation.w = pose.orientation.w_val
            transformList.append(static_transformStamped)
            time.sleep(1)
            rospy.loginfo("Started static transform for LiDAR sensor with ID " + sensor_name + ".")

        for sensor_name in cpulidar_names:
            lidar_data = client.getLidarData(sensor_name, vehicle_name)
            pose = lidar_data.pose
            static_transformStamped = geometry_msgs.msg.TransformStamped()
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = base_frame
            static_transformStamped.child_frame_id = str(lidar_frame_prefix) + "_" + str(get_valid_ros_topic_nameid(sensor_name))
            static_transformStamped.transform.translation.x = pose.position.x_val
            static_transformStamped.transform.translation.y = -pose.position.y_val
            static_transformStamped.transform.translation.z = pose.position.z_val
            static_transformStamped.transform.rotation.x = pose.orientation.x_val
            static_transformStamped.transform.rotation.y = pose.orientation.y_val
            static_transformStamped.transform.rotation.z = pose.orientation.z_val
            static_transformStamped.transform.rotation.w = pose.orientation.w_val
            transformList.append(static_transformStamped)
            time.sleep(1)
            rospy.loginfo("Started static transform for LiDAR sensor with ID " + sensor_name + ".")

        broadcaster.sendTransform(transformList)

        airsim_echos_pub(ros_rate, vehicle_name, echo_topic_prefix, echo_topic_suffix, lidar_topic_prefix,
                         lidar_topic_suffix, object_topic_prefix, object_topic_suffix,
                         echo_frame_prefix, lidar_frame_prefix, base_frame,
                         echo_names, lidar_names, cpulidar_names, object_names)

    except rospy.ROSInterruptException:
        pass
