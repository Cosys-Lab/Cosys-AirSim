# Python client example to get lidar data 
#

import setup_path
import cosysairsim as airsim
import numpy as np
import matplotlib.pyplot as plt


def lidar_rings(number_of_points, number_of_channels, gpulidar):
    # Laser (ring) index of each point, 0 is the lowest laser. Both lidars give one point per laser for each horizontal
    # step, the GPU lidar starts with the lowest laser and the CPU lidar with the highest one.
    # This matches the ring field of the point clouds published by the ROS2 node.
    index = np.arange(number_of_points) % number_of_channels
    return index if gpulidar else number_of_channels - 1 - index


class lidarTest:

    def __init__(self, lidar_name, vehicle_name):

        # connect to the AirSim simulator
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.vehicleName = vehicle_name
        self.lidarName = lidar_name
        self.lastlidarTimeStamp = 0

    def get_data(self, gpulidar):

        # # get lidar data
        if gpulidar:
            lidarData = self.client.getGPULidarData(self.lidarName, self.vehicleName)

            if lidarData.time_stamp != self.lastlidarTimeStamp:
                # Check if there are any points in the data
                if len(lidarData.point_cloud) < 5:
                    self.lastlidarTimeStamp = lidarData.time_stamp
                    return None
                else:
                    self.lastlidarTimeStamp = lidarData.time_stamp

                    points = np.array(lidarData.point_cloud, dtype=np.dtype('f4'))
                    points = np.reshape(points, (int(points.shape[0] / 5), 5))
                    print("got " + str(points.shape[0]) + " points")
                    return points
            else:
                return None
        else:
            lidarData = self.client.getLidarData(self.lidarName, self.vehicleName)

            if lidarData.time_stamp != self.lastlidarTimeStamp:
                # Check if there are any points in the data
                if len(lidarData.point_cloud) < 5:
                    self.lastlidarTimeStamp = lidarData.time_stamp
                    return None
                else:
                    self.lastlidarTimeStamp = lidarData.time_stamp

                    points = np.array(lidarData.point_cloud, dtype=np.dtype('f4'))
                    points = np.reshape(points, (int(points.shape[0] / 3), 3))
                    points = points * np.array([1, -1, -1])
                    print("got " + str(points.shape[0]) + " points")
                    return points
            else:
                return None

    def stop(self):

        self.client.reset()
        print("Stopped!\n")


# main
if __name__ == "__main__":

    lidarTest = lidarTest('lidar2', 'airsimvehicle')
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    gpulidar = True
    number_of_channels = 32  # NumberOfChannels of the lidar in settings.json

    points = lidarTest.get_data(gpulidar)
    rings = lidar_rings(points.shape[0], number_of_channels, gpulidar)
    print("rings " + str(rings.min()) + " to " + str(rings.max()))
    ax.set_box_aspect((np.ptp(points[:, 0]), np.ptp(points[:, 1]), np.ptp(points[:, 2])))
    ax.scatter(points[:, 0], points[:, 1], points[:, 2])
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.set_xlim((-5, 5))
    ax.set_ylim((-5, 5))
    plt.show()
