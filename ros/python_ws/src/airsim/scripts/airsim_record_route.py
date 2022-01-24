#!/usr/bin/env python

import setup_path
import airsimpy
import rospy
import time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped


def airsim_record_route(ros_rate, ip, toggle_drone, vehicle_name, pose_topic, pose_frame, sensor_imu_enable,
                        sensor_imu_name, sensor_imu_topic, sensor_imu_frame):

    rate = rospy.Rate(ros_rate)

    rospy.loginfo("Connecting to AirSim...")
    if toggle_drone:
        client = airsimpy.MultirotorClient(ip)
    else:
        client = airsimpy.CarClient(ip)
    client.confirmConnection(rospy.get_name())
    rospy.loginfo("Connected to AirSim!")

    pose_publisher = rospy.Publisher(pose_topic, PoseStamped, queue_size=1)
    if sensor_imu_enable:
        imu_publisher = rospy.Publisher(sensor_imu_topic, Imu, queue_size=1)

    rospy.loginfo("Started publishers...")

    while not rospy.is_shutdown():

        timestamp = rospy.Time.now()

        pose = client.simGetVehiclePose(vehicle_name)
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

        if sensor_imu_enable:
            imu_data = client.getImuData(sensor_imu_name, vehicle_name)

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

        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('airsim_record_route', anonymous=True)

        ip = rospy.get_param('~ip', "")

        ros_rate = rospy.get_param('~rate', 100)

        toggle_drone = rospy.get_param('~toggle_drone', 0)

        vehicle_name = rospy.get_param('~vehicle_name', 'airsimvehicle')

        pose_topic = rospy.get_param('~pose_topic', "airsim/gtpose")
        pose_frame = rospy.get_param('~pose_frame', "world")

        sensor_imu_enable = rospy.get_param('~sensor_imu_enable', 1)
        sensor_imu_name = rospy.get_param('~sensor_imu_name', "imu")
        sensor_imu_topic = rospy.get_param('~sensor_imu_topic', "airsim/imu")
        sensor_imu_frame = rospy.get_param('~sensor_imu_frame', "base_link")

        airsim_record_route(ros_rate, ip, toggle_drone, vehicle_name, pose_topic, pose_frame, sensor_imu_enable,
                            sensor_imu_name, sensor_imu_topic, sensor_imu_frame)

    except rospy.ROSInterruptException:
        pass
