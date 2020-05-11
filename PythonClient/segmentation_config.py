#!/usr/bin/env python

import setup_path
import airsimpy
import csv

if __name__ == '__main__':
    client = airsimpy.CarClient()
    client.confirmConnection()

    list = client.simListSceneObjects()
    with open('allObjectsFound.txt', 'w') as f:
        for item in list:
            f.write("%s\n" % item)

    with open('segmentation.csv', encoding='utf-8-sig') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        index = 0
        for idx, row in enumerate(csv_reader, start=1):
            success = client.simSetSegmentationObjectID(row[0] + ".*", idx, True)
            if success:
                print("Found objects matching object-name '" + str(row[0]) + "'! Set segmentation value " + str(idx))
            else:
                print("No objects found matching object-name '" + str(row[0]) + ".")