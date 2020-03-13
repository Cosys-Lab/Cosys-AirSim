#!/usr/bin/env python

import setup_path
import airsimpy
import csv
import random
import numpy as np
from PIL import Image

# Demonstration script to show how the instance segmentation can be accessed and updated through the API
# See docs/instance_segmentation.md and docs/image_apis.md#segmentation for more information.
if __name__ == '__main__':

    # Make connection to AirSim API
    client = airsimpy.CarClient()
    client.confirmConnection()

    # Get names of all objects in simulation world and store in list
    # In a dynamic world, this is never the same!!
    currentObjectList = client.simListSceneObjects()
    print("Generating list of all current objects...")
    with open('allObjectsFound.txt', 'w') as f:
        for item in currentObjectList:
            f.write("%s\n" % item)
    print("Generated list of all current objects with a total of " + str(len(currentObjectList)) + ' objects\n')

    # Sort the objects from the list by class defined in the CSV and keep them in a dictionary with classname as key
    print("Sorting objects based on segmentation.csv into classes...")
    with open('segmentation.csv', encoding='utf-8-sig') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        index = 0
        classObjectMap = {}
        for idx, row in enumerate(csv_reader, start=1):
            classItems = [i for i in currentObjectList if i.startswith(row[0])]
            print("Found " + str(len(classItems)) + " objects starting with object-name '" + str(row[0]) + "'")
            classObjectMap[row[0]] = classItems
    print("Sorted objects based on segmentation.csv into classes\n")

    # Set the colors for all AI humans to a chosen color with color index 15
    className = 'wall'
    classColorIndex = 1000000
    # a) this version we set it based on the gathered objects in the list
    print("Setting all objects in world matching class-name '" + className + "' a certain color, based on object list...")
    for item in classObjectMap[className]:
        success = client.simSetSegmentationObjectID(item, classColorIndex, False)
        if success:
            print("Found object matching object-name '" + item + "'! Set segmentation value " + str(classColorIndex))
        else:
            print("No object found matching object-name '" + item + "'")
    print("Found and changed color on all objects in world matching class-name '" + className + "' based on object list\n")
    # b) In this version we set it based on regex of the classname
    print("Setting all objects in world matching class-name '" + className + "' a certain color, based on regex...")
    success = client.simSetSegmentationObjectID(className + ".*", classColorIndex, True)
    if success:
        print("Found objects matching object-name '" + className + "'! Set segmentation value " + str(classColorIndex))
    else:
        print("No objects found matching object-name '" + className + "'")
    print("Found and changed color on all objects in world matching class-name '" + className + "' based on regex\n")

    # Generate list of all colors available for segmentation
    print("Generating segmentation colormap, this takes a while...")
    colorMap = client.simGetSegmentationColorMap()
    print("Generated segmentation colormap\n")

    # Get the color associated with a the class
    classColor = colorMap[classColorIndex,:]
    print("Found RGB color associated with class '" + className + "': " + str(classColor) + "\n")

    # Get the color associated with a random object from the list
    randomObjectName = random.choice(currentObjectList)
    print("Finding RGB color associated with random object '" + randomObjectName + "'...")
    randomObjectColorIndex = client.simGetSegmentationObjectID(randomObjectName)
    randomObjectColor = colorMap[randomObjectColorIndex,:]
    print("Found RGB color associated with random object '" + randomObjectName + "':" + str(randomObjectColor) + "\n")

    # Get an image from the main segmentation camera, show and save it as png
    print("Getting segmentation image from main camera...")
    responses = client.simGetImages([airsimpy.ImageRequest( "front_center", airsimpy.ImageType.Segmentation, False, False)])
    img_rgb_string = responses[0].image_data_uint8
    rgbarray = np.frombuffer(img_rgb_string, np.uint8)
    rgbarray_shaped = rgbarray.reshape((540,960,3))
    rgbarray_shaped = rgbarray_shaped
    img = Image.fromarray(rgbarray_shaped, 'RGB')
    img.show()
    img.save("segmentation_sample.png", "PNG")
    print("Shown and saved segmentation image from main camera\n")

    print("All instance segmentation tests completed")
