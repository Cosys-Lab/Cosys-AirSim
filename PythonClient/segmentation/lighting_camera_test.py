#!/usr/bin/env python

import setup_path
import cosysairsim as airsim
import csv
import random
import numpy as np
from PIL import Image
from datetime import datetime
import matplotlib.pyplot as plt


if __name__ == '__main__':

    client = airsim.CarClient()
    client.confirmConnection()

    responses = client.simGetImages([airsim.ImageRequest( "frontcamera", airsim.ImageType.Scene, False, False)])
    img_rgb_string = responses[0].image_data_uint8
    rgbarray = np.frombuffer(img_rgb_string, np.uint8)
    rgbarray_shaped = rgbarray.reshape((540,960,3))
    # img = Image.fromarray(rgbarray_shaped, 'RGB')
    # img.show()

    # 3. Create Figure and Subplots using Matplotlib
    fig, axes = plt.subplots(1, 2, figsize=(15, 6))  # 1 row, 2 columns. Adjust figsize as needed.

    # Plot Scene Image
    axes[0].imshow(rgbarray_shaped)
    axes[0].set_title('Scene')
    axes[0].axis('off')  # Hide axes ticks and labels


    responses = client.simGetImages([airsim.ImageRequest( "frontcamera", airsim.ImageType.Lighting, False, False)])
    img_rgb_string = responses[0].image_data_uint8
    rgbarray = np.frombuffer(img_rgb_string, np.uint8)
    rgbarray_shaped = rgbarray.reshape((540,960,3))
    # img = Image.fromarray(rgbarray_shaped, 'RGB')
    # img.show()


    # Plot Lighting Image
    axes[1].imshow(rgbarray_shaped)
    axes[1].set_title('Lightning')  # Changed from "Lighting" based on your prompt title request
    axes[1].axis('off')  # Hide axes ticks and labels

    # Adjust layout to prevent titles overlapping
    plt.tight_layout()

    # Show the combined plot
    plt.show()



