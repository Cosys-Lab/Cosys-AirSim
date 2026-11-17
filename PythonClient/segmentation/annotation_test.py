#!/usr/bin/env python

import setup_path
import cosysairsim as airsim
import csv
import random
import re
import numpy as np
from PIL import Image
from datetime import datetime

if __name__ == '__main__':

    client = airsim.CarClient()
    client.confirmConnection()

    annotation_name = "GreyscaleTest"
    annotation_objects = client.simListAnnotationObjects(annotation_name)
    assert annotation_objects, "GreyscaleTest has no annotated objects"
    mesh_name = annotation_objects[0]
    original_value = client.simGetAnnotationObjectValue(annotation_name, mesh_name)
    assert client.simSetAnnotationObjectValue(annotation_name, mesh_name, original_value)
    regex = "^" + re.escape(mesh_name) + "$"
    assert client.simSetAnnotationObjectValue(annotation_name, regex, original_value, True)
    assert abs(client.simGetAnnotationObjectValue(annotation_name, mesh_name) - original_value) < 1e-3

    responses = client.simGetImages([airsim.ImageRequest( "front_center", airsim.ImageType.Segmentation, False, False)])
    img_rgb_string = responses[0].image_data_uint8
    rgbarray = np.frombuffer(img_rgb_string, np.uint8)
    rgbarray_shaped = rgbarray.reshape((540,960,3))
    img = Image.fromarray(rgbarray_shaped, 'RGB')
    img.show()
    img.save("segmentation.png", "PNG")

    responses = client.simGetImages([airsim.ImageRequest( "front_center", airsim.ImageType.Annotation, False, False, "TextureTestDirect")])
    img_rgb_string = responses[0].image_data_uint8
    rgbarray = np.frombuffer(img_rgb_string, np.uint8)
    rgbarray_shaped = rgbarray.reshape((540,960,3))
    img = Image.fromarray(rgbarray_shaped, 'RGB')
    img.show()
    img.save("TextureTestDirect.png", "PNG")

    responses = client.simGetImages([airsim.ImageRequest( "front_center", airsim.ImageType.Annotation, False, False, "RGBTestIndex")])
    img_rgb_string = responses[0].image_data_uint8
    rgbarray = np.frombuffer(img_rgb_string, np.uint8)
    rgbarray_shaped = rgbarray.reshape((540,960,3))
    img = Image.fromarray(rgbarray_shaped, 'RGB')
    img.show()
    img.save("RGBTestIndex.png", "PNG")

    responses = client.simGetImages([airsim.ImageRequest( "front_center", airsim.ImageType.Annotation, False, False, "RGBTestDirect")])
    img_rgb_string = responses[0].image_data_uint8
    rgbarray = np.frombuffer(img_rgb_string, np.uint8)
    rgbarray_shaped = rgbarray.reshape((540,960,3))
    img = Image.fromarray(rgbarray_shaped, 'RGB')
    img.show()
    img.save("RGBTestDirect.png", "PNG")

    responses = client.simGetImages([airsim.ImageRequest( "front_center", airsim.ImageType.Annotation, False, False, "GreyscaleTest")])
    img_rgb_string = responses[0].image_data_uint8
    rgbarray = np.frombuffer(img_rgb_string, np.uint8)
    rgbarray_shaped = rgbarray.reshape((540,960,3))
    greyscale_values = np.divide(rgbarray_shaped[:,:,0], 255)
    img = Image.fromarray(rgbarray_shaped[:,:,0])
    img.show()
    img.save("GreyscaleTest.png", "PNG")

    responses = client.simGetImages([airsim.ImageRequest( "front_center", airsim.ImageType.Annotation, False, False, "TextureTestRelativePath")])
    img_rgb_string = responses[0].image_data_uint8
    rgbarray = np.frombuffer(img_rgb_string, np.uint8)
    rgbarray_shaped = rgbarray.reshape((540,960,3))
    img = Image.fromarray(rgbarray_shaped, 'RGB')
    img.show()
    img.save("TextureTestRelativePath.png", "PNG")
