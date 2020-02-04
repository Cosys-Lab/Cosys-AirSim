import os
import sys
import keyboard
import shutil
import time
import numpy as np
import pandas as pd
import json
import jsonpickle
import ArmClient

import setup_path
import airsim

def main():
    client = ArmClient.ArmClient()
    output_folder_name = 'arm_out'
    camera_names = ['aFollow', 'bArm']
    lidar_names = []


    print('Setting first pose.')
    client.set_arm_pose(0.25, 0.5, 0.8, 1, 0.5)
    time.sleep(5)

    print('Setting second pose.')
    client.set_arm_pose(0.75, 0.5, 0.8, 1, 0.5)
    time.sleep(5)

    print('Closing claw.')
    client.set_manipulator(0)
    time.sleep(5)

    print('Opening claw.')
    client.set_manipulator(1)
    time.sleep(5)

    return

main()
