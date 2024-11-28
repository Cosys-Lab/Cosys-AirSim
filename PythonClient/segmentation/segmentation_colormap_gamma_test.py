import cosysairsim as airsim
import numpy as np
import json

# Place large cube in environment right in front of a camera called frontcamera on vehicle called airsimvehicle
# so that the cube is visible in the camera image in the center pixel

# Connect to simulation
client = airsim.CarClient()
client.confirmConnection()
client.reset()

# Find unique color values
colormap = client.simGetSegmentationColorMap()
uniqueColors = np.unique(colormap[:, 0])
uniqueColorIndexes = np.zeros((len(uniqueColors), 1), dtype=np.int32)
for i in range(0, len(uniqueColors)):
    FoundIndex = np.where((colormap == np.array([uniqueColors[i], uniqueColors[i], uniqueColors[i]])).all(axis=1))[0]
    uniqueColorIndexes[i] = int(FoundIndex[0])

# Loop those colors
wrongColors = {}
imageRequest = airsim.ImageRequest("frontcamera", airsim.ImageType.Segmentation, False, False)
for i in range(0, len(uniqueColorIndexes)):

    # Set that color
    color = colormap[int(uniqueColorIndexes[i]), :].astype(np.uint8)
    print("\nsettings index " + str(uniqueColorIndexes[i]) + " which should give color " + str(color))
    client.simSetSegmentationObjectID('StaticMeshActor_1', int(uniqueColorIndexes[i]))

    # Get a instance segmentation image and parse the middle pixel color
    imageResponse = client.simGetImages([imageRequest], "airsimvehicle")
    img_rgb_string = imageResponse[0].image_data_uint8
    rgbarray = np.frombuffer(img_rgb_string, np.uint8)
    image = rgbarray.reshape((imageResponse[0].height, imageResponse[0].width, 3))
    colorInMiddle = image[imageResponse[0].height // 2, imageResponse[0].width // 2]
    print("Color found: " + str(colorInMiddle))

    # Compare and if wrong store it
    if not np.array_equal(colorInMiddle.astype(np.uint8), color):
        print("Color in middle does not match expected color!")
        if colorInMiddle[0] != color[0]:
            if not str(color[0]) in wrongColors:
                wrongColors[str(color[0])] = [colorInMiddle[0]]
            else:
                if not colorInMiddle[0] in wrongColors[str(color[0])]:
                    wrongColors[str(color[0])].append(colorInMiddle[0])
        elif colorInMiddle[1] != color[1]:
            if not str(color[1]) in wrongColors:
                wrongColors[str(color[1])] = [colorInMiddle[1]]
            else:
                if not colorInMiddle[1] in wrongColors[str(color[1])]:
                    wrongColors[str(color[1])].append(colorInMiddle[1])
        elif colorInMiddle[2] != color[2]:
            if not str(color[2]) in wrongColors:
                wrongColors[str(color[2])] = [colorInMiddle[2]]
            else:
                if not colorInMiddle[2] in wrongColors[str(color[2])]:
                    wrongColors[str(color[2])].append(colorInMiddle[2])

print("wrongcolors:" + str(wrongColors))