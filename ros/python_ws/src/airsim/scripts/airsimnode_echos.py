#!/usr/bin/env python


import time
import setup_path
import airsimpy
import rospy
from std_msgs.msg import String, Header
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import tf2_ros
import geometry_msgs


print("Connecting...")
client = airsimpy.CarClient()
client.confirmConnection(rospy.get_name())
print("Connected")


def get_valid_ros_topic_nameid(sensor_name):
    sensor_name_fixed = sensor_name.replace("-","_")
    sensor_name_fixed = sensor_name_fixed.replace("__", "_")
    sensor_name_fixed = sensor_name_fixed.replace("//", "/")
    if sensor_name_fixed[0].isdigit():
        sensor_name_fixed = "a" + sensor_name_fixed
    return sensor_name_fixed


def airsim_echos_pub(ros_rate, vehicle_name, topic_prefix, topic_suffix, frame_prefix, sensor_names):

    rospy.Rate(ros_rate)

    pointcloud_publishers = {}
    last_timestamps = {}
    for sensor_name in sensor_names:
        pointcloud_publishers[sensor_name] = rospy.Publisher(topic_prefix + "/"
                                                             + str(get_valid_ros_topic_nameid(sensor_name))
                                                             + "/" + topic_suffix, PointCloud2, queue_size=2)
        last_timestamps[sensor_name] = None

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('a', 12, PointField.FLOAT32, 1),
        PointField('d', 16, PointField.FLOAT32, 1)
    ]

    while not rospy.is_shutdown():

        timestamp = rospy.Time.now()

        for sensor_name in sensor_names:
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
                    header.frame_id = frame_prefix + "_" + str(get_valid_ros_topic_nameid(sensor_name))
                    pcloud = pc2.create_cloud(header, fields, points_list)
                    pcloud.header.stamp = timestamp

                    pointcloud_publishers[sensor_name].publish(pcloud)


if __name__ == '__main__':
    try:
        rospy.init_node('airsim_echos_publisher', anonymous=True)

        ros_rate = rospy.get_param('~ros_rate', 10)

        topic_prefix = rospy.get_param('~topic_prefix', "rtis")
        topic_suffix = rospy.get_param('~topic_suffix', "pointcloud")
        vehicle_name = rospy.get_param('~vehicle_name', 'airsimvehicle')
        frame_prefix = rospy.get_param('~frame_prefix', "base_rtis")
        base_frame = rospy.get_param('~base_frame', "world")

        sensor_names = rospy.get_param('~sensor_names', "True")

        broadcaster = tf2_ros.StaticTransformBroadcaster()
        transformList = []
        for sensor_name in sensor_names:
            echo_data = client.getEchoData(sensor_name, vehicle_name)
            pose = echo_data.pose

            static_transformStamped = geometry_msgs.msg.TransformStamped()
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = base_frame
            static_transformStamped.child_frame_id = str(frame_prefix) + "_" + str(get_valid_ros_topic_nameid(sensor_name))
            static_transformStamped.transform.translation.x = pose.position.x_val
            static_transformStamped.transform.translation.y = pose.position.y_val
            static_transformStamped.transform.translation.z = pose.position.z_val
            static_transformStamped.transform.rotation.x = pose.orientation.x_val
            static_transformStamped.transform.rotation.y = pose.orientation.y_val
            static_transformStamped.transform.rotation.z = pose.orientation.z_val
            static_transformStamped.transform.rotation.w = pose.orientation.w_val
            transformList.append(static_transformStamped)
            time.sleep(1)
            rospy.loginfo("Started static tranform for RTIS Client with ID " + sensor_name + ".")
        broadcaster.sendTransform(transformList)

        airsim_echos_pub(ros_rate, vehicle_name, topic_prefix, topic_suffix, frame_prefix, sensor_names)

    except rospy.ROSInterruptException:
        pass
