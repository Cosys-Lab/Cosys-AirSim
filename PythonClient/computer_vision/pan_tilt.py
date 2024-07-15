import setup_path
import cosysairsim as airsim
import math
import time
import numpy as np

# Connect to AirSim
client = airsim.VehicleClient()
client.confirmConnection()

for pan_angle in np.arange(-math.pi/2, (math.pi/2)+math.pi/180, math.pi/180):
    for tilt_angle in np.arange(-math.pi/6, (math.pi/6)+math.pi/180, math.pi/180):
        print("Pan:", round(np.rad2deg(pan_angle)),"Tilt:", round(np.rad2deg(tilt_angle)))
        client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(tilt_angle, 0, pan_angle)), True)
        time.sleep(0.1)

