# Constructing the Python Client

# Overview
Once the URDF XML is written and the settings file is updated, the final step is to create the client for interacting with the robot. AirSim provides two mechanisms for controlling the motion of the robot: Controlled Motion Components and Forces. 

## Controlled Motion Components
Controlled Motion Components are commonly-used actuation components that appear in many different kinds of robots. This is the simplest and most robust manner of controlling the robot. To create these types of controlled motion components, add the corresponding <limit> tag to the joint as described in [Construct an XML file](UrdfXml.md). There are three types of controlled motion components that can be created:

* **Servo**: This is created from a revolute joint. This represents controlled positional motion around a single axis. 
* **Linear Actuator**: This is created from a prismatic joint. This represents controlled positional motion along a single axis. 
* **Motor**: This is created from a continuous joint. This represents controlled velocity around a single axis.

In each case, the "effort" attribute in the URDF XML specifies the amount of force / torque the component can apply. For Servos and Linear Actuators, the "velocity" member will control the maximum velocity of the component. To set the target position, use the API call updateControlledMotionComponentControlSignal, passing a value between 0 and 1 as the control signal. 0 corresponds to a fully retracted actuator / fully left-facing servo / fully backwards-running motor, while 1 corresponds to a fully extended / fully right-facing servo / fully forward running motor. To see a full example of using a controlled motion component, see the client at PythonClient/urdfbot/LunabotClient.py.

## Arbitrary Forces

For some robots, the motion requirements are more complex than simple linear actuators and servos. For this scenario, it's possible to add arbitrary forces and torques to bodies in AirSim. This is a two step process. First, the API call AddLinearForce / AddAngularForce is used to register the force with the simulation engine. Once the force is registered, then the force's name can be passed as a parameter to updateForceMagnitude. For a full example, see the client at PythonClient/urdfbot/BoxAndCylinderClient.py (no URDF XML provided for this example).