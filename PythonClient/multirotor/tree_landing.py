#!/usr/bin/env python3
import setup_path 
import cosysairsim as airsim

import sys
import time

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

landed = client.getMultirotorState().landed_state
if landed == airsim.LandedState.Landed:
    print("taking off...")
    client.takeoffAsync().join()
else:
    client.hoverAsync().join()

# AirSim uses NED coordinates so negative axis is up.
# z of -7 is 7 meters above the original launch point.
z = -30

# see https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo

# this method is async and we are not waiting for the result since we are passing timeout_sec=0.
result = client.moveOnPathAsync([airsim.Vector3r(-80,-120,z),airsim.Vector3r(60,-120,z),airsim.Vector3r(-56,-58,z)], 
                        8, 120, 
                        airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), 20, 1).join()
client.moveToPositionAsync(-56,-58,0,2).join()
print("moved")
client.armDisarm(False)
client.enableApiControl(False)

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

landed = client.getMultirotorState().landed_state
if landed == airsim.LandedState.Landed:
    print("already landed...")
else:
    print("landing...")
    client.landAsync().join()

client.armDisarm(False)
client.enableApiControl(False)
