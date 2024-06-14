#!/usr/bin/env python

import setup_path
import cosysairsim as airsim
import csv
import random
import numpy as np
from PIL import Image
from datetime import datetime
import matplotlib.pyplot as plt


# Demonstration script to show how the camera and GPU-LiDAR sensor can be used and their data retrieved.
# See docs/instance_segmentation.md,docs/gpu-lidar.md and docs/image_apis.md#segmentation for more information.
if __name__ == '__main__':

    # Make connection to AirSim API
    client = airsim.CarClient()
    client.confirmConnection()


    # Get an image from the main segmentation camera, show and save it as png
    print("Getting segmentation image from main camera...")
    responses = client.simGetImages([airsim.ImageRequest( "frontcamera", airsim.ImageType.Segmentation, False, False)])
    img_rgb_string = responses[0].image_data_uint8
    rgbarray = np.frombuffer(img_rgb_string, np.uint8)
    rgbarray_shaped = rgbarray.reshape((540,960,3))
    img = Image.fromarray(rgbarray_shaped, 'RGB')
    img.show()
    img.save("segmentation_sample.png", "PNG")
    print("Shown and saved segmentation image from main camera\n")

    # Get an image from the main segmentation camera, show and save it as png
    print("Getting rgb image from main camera...")
    responses = client.simGetImages([airsim.ImageRequest( "frontcamera", airsim.ImageType.Scene, False, False)])
    img_rgb_string = responses[0].image_data_uint8
    rgbarray = np.frombuffer(img_rgb_string, np.uint8)
    rgbarray_shaped = rgbarray.reshape((540,960,3))
    img = Image.fromarray(rgbarray_shaped, 'RGB')
    img.show()
    img.save("rgb_sample.png", "PNG")
    print("Shown and saved RGB image from main camera\n")

    # Get an image from the main segmentation camera, show and save it as png
    print("Getting depth image from main camera...")
    responses = client.simGetImages([airsim.ImageRequest( "frontcamera", airsim.ImageType.DepthPlanar, True, False)])
    img_depth_float = responses[0].image_data_float
    img_depth_float32 = np.asarray(img_depth_float, dtype=np.float32)
    rgbarray_shaped = img_depth_float32.reshape((540,960)).astype(np.float32)
    img = Image.fromarray(rgbarray_shaped)
    img.show()
    img.save("depth_sample.tif")
    print("Shown and saved depth image from main camera\n")

    print("Getting GPU LiDAR data...")
    lidar_data = client.getGPULidarData('gpulidar', 'airsimvehicle')
    points = np.array(lidar_data.point_cloud, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 5), 5))

    rgb_values = points[:, 3].astype(np.uint32)
    rgb = np.zeros((np.shape(points)[0], 3))
    xyz = points[:, 0:3]
    for index, rgb_value in enumerate(rgb_values):
        rgb[index, 0] = (rgb_value >> 16) & 0xFF
        rgb[index, 1] = (rgb_value >> 8) & 0xFF
        rgb[index, 2] = rgb_value & 0xFF
    rgb = np.divide(rgb, 255)

    print("Parsing segmentation RGB8 values from float32 data values...")
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(xyz[:, 0], -xyz[:, 1], -xyz[:, 2], c=rgb)
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.set_xlim((-50, 50))
    ax.set_ylim((-50, 50))
    ax.set_zlim((-50, 50))
    plt.show()
    print("Shown and saved 3d segmented pointcloud\n")

    print("All instance segmentation tests completed")