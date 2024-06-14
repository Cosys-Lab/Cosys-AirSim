#!/usr/bin/env python

import setup_path
import airsim
import csv
import random
import numpy as np
from PIL import Image
from datetime import datetime

if __name__ == '__main__':

    # Make connection to AirSim API
    client = airsim.CarClient()
    client.confirmConnection()

    responses = client.simGetImages([airsim.ImageRequest( "front_center", airsim.ImageType.Annotation, False, False, "RGBTestDirect")])
    img_rgb_string = responses[0].image_data_uint8
    rgbarray = np.frombuffer(img_rgb_string, np.uint8)
    rgbarray_shaped = rgbarray.reshape((540,960,3))
    img = Image.fromarray(rgbarray_shaped, 'RGB')
    img.show()

    responses = client.simGetImages([airsim.ImageRequest( "front_center", airsim.ImageType.Annotation, False, False, "GreyscaleTest")])
    img_rgb_string = responses[0].image_data_uint8
    rgbarray = np.frombuffer(img_rgb_string, np.uint8)
    rgbarray_shaped = rgbarray.reshape((540,960,3))
    greyscale_values = np.divide(rgbarray_shaped[:,:,0], 255)
    img = Image.fromarray(rgbarray_shaped[:,:,0])
    img.show()

    responses = client.simGetImages([airsim.ImageRequest( "front_center", airsim.ImageType.Annotation, False, False, "TextureTestRelativePath")])
    img_rgb_string = responses[0].image_data_uint8
    rgbarray = np.frombuffer(img_rgb_string, np.uint8)
    rgbarray_shaped = rgbarray.reshape((540,960,3))
    img = Image.fromarray(rgbarray_shaped, 'RGB')
    img.show()

