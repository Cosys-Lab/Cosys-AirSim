#!/usr/bin/env python

import setup_path
import cosysairsim as airsim
import csv
import random
import numpy as np
from PIL import Image
from datetime import datetime

# Demonstration script to show how the instance segmentation can be accessed and updated through the API
# See docs/instance_segmentation.md and docs/image_apis.md#segmentation for more information.
if __name__ == '__main__':

    # Make connection to AirSim API
    client = airsim.CarClient()
    client.confirmConnection()

    # Generate list of all colors available for segmentation
    print("Getting segmentation colormap...")
    colorMap = client.simGetSegmentationColorMap()
    print("Getting segmentation colormap\n")

    # Get names of all objects in simulation world in the instance segmentation format
    # and store in list together with the associated RGB value
    # In a dynamic world, this is never the same!!
    currentObjectList = client.simListInstanceSegmentationObjects()
    print("Generating list of all current objects...")
    with open('airsim_segmentation_colormap_list_' +  datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + '.csv', 'w') as f:
        f.write("ObjectName,R,G,B\n")
        for index, item in enumerate(currentObjectList):
            f.write("%s,%s\n" % (item, ','.join([str(x) for x in colorMap[index,:]])))
    print("Generated list of all current objects with their RGB value with a total of " + str(len(currentObjectList)) + ' objects\n')

    # Get names of all objects in simulation world in the instance segmentation format
    # and store in list together with the object 3D pose
    currentPosesList = client.simListInstanceSegmentationPoses()
    print("Generating list of all current objects poses...")
    with open('airsim__poses_list_' +  datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + '.csv', 'w') as f:
        f.write("ObjectName,x_pos,y_pos,z_pos,w_qua,x_qua,y_qua,z_qua\n")
        for index, item in enumerate(currentObjectList):
            currentObjectPose = currentPosesList[index]
            currentObjectPoseString = (str(currentObjectPose.position.x_val) + "," +
                                       str(currentObjectPose.position.y_val) + "," +
                                       str(currentObjectPose.position.z_val) + "," +
                                       str(currentObjectPose.orientation.w_val) + "," +
                                       str(currentObjectPose.orientation.x_val) + "," +
                                       str(currentObjectPose.orientation.y_val) + "," +
                                       str(currentObjectPose.orientation.z_val))
            f.write("%s,%s\n" % (item, currentObjectPoseString))
    print("Generated list of all current objects with their poses a total of " + str(len(currentObjectList)) + ' objects\n')

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
    className = 'ground'
    classColorIndex = 1000000
    # a) this version we set it based on the gathered objects in the list
    print("Setting all objects in world matching class-name '"
          + className + "' a certain color, based on object list...")
    for item in classObjectMap[className]:
        success = client.simSetSegmentationObjectID(item, classColorIndex, False)
        if success:
            print("Found object matching object-name '" + item + "'! Set segmentation value " + str(classColorIndex))
        else:
            print("No object found matching object-name '" + item + "'")
    print("Found and changed color on all objects in world matching class-name '"
          + className + "' based on object list\n")
    # b) In this version we set it based on regex of the classname
    print("Setting all objects in world matching class-name '" + className + "' a certain color, based on regex...")
    success = client.simSetSegmentationObjectID(className + ".*", classColorIndex, True)
    if success:
        print("Found objects matching object-name '" + className + "'! Set segmentation value " + str(classColorIndex))
    else:
        print("No objects found matching object-name '" + className + "'")
    print("Found and changed color on all objects in world matching class-name '" + className + "' based on regex\n")

    # Get the color associated with a the class
    classColor = colorMap[classColorIndex,:]
    print("Found RGB color associated with class '" + className + "': " + str(classColor) + "\n")

    # Get the color associated with a random object from the list
    randomObjectName = random.choice(currentObjectList)
    print("Finding RGB color associated with random object '" + randomObjectName + "'...")
    randomObjectColorIndex = client.simGetSegmentationObjectID(randomObjectName)
    randomObjectColor = colorMap[randomObjectColorIndex, :]
    print("Found RGB color associated with random object '" + randomObjectName + "':" + str(randomObjectColor) + "\n")

    # Get an image from the main segmentation camera, show and save it as png
    print("Getting segmentation image from main camera...")
    responses = client.simGetImages([airsim.ImageRequest( "frontcamera", airsim.ImageType.Segmentation, False, False)])
    img_rgb_string = responses[0].image_data_uint8
    rgbarray = np.frombuffer(img_rgb_string, np.uint8)
    rgbarray_shaped = rgbarray.reshape((540,960,3))
    rgbarray_shaped = rgbarray_shaped
    img = Image.fromarray(rgbarray_shaped, 'RGB')
    img.show()
    img.save("segmentation_sample.png", "PNG")
    print("Shown and saved segmentation image from main camera\n")

    print("All instance segmentation tests completed")
