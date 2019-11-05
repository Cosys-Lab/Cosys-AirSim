#!/usr/bin/env python

import setup_path 
import airsim
import rospy
import geometry_msgs.msg


def handle_input_command(msg, args):

     # set the controls for car
    car_controls = airsim.CarControls()
    if (msg.linear.x < 0):
        car_controls.is_manual_gear = True
        car_controls.manual_gear = -1
    else:
        car_controls.is_manual_gear = False
    car_controls.throttle = msg.linear.x
    car_controls.steering = -msg.angular.z
    args[1].setCarControls(car_controls, args[0])


if __name__ == '__main__':
    try:
        rospy.init_node('airsim_car_control')
        inputTopic = rospy.get_param('~input_topic', 'cmd_vel')
        vehicleName = rospy.get_param('~vehicle_name', 'vehicle')

        # connect to the AirSim simulator 
        client = airsim.CarClient()
        client.confirmConnection()
        client.enableApiControl(True)
        rospy.Subscriber(inputTopic, geometry_msgs.msg.Twist, handle_input_command, (vehicleName, client))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
