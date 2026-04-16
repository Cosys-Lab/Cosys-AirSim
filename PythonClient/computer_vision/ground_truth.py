# In settings.json first activate computer vision mode:
# https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/docs/image_apis.md#computer-vision-mode

import pprint
import time

import cv2  # conda install opencv

import cosysairsim as airsim

client = airsim.VehicleClient()
client.confirmConnection()

print("Time,Speed,Gear,PX,PY,PZ,OW,OX,OY,OZ")

# monitor car state while you drive it manually.
while (cv2.waitKey(1) & 0xFF) == 0xFF:
    kinematics = client.simGetGroundTruthKinematics()
    environment = client.simGetGroundTruthEnvironment()

    print(f"Kinematics: {pprint.pformat(kinematics)}\nEnvironemt {pprint.pformat(environment)}")
    time.sleep(1)
