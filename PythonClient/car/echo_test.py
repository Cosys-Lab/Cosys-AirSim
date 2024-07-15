# Python client example to get echo data 
#

import setup_path
import cosysairsim as airsim
import numpy as np
import matplotlib.pyplot as plt

class EchoTest:

    def __init__(self, echo_name, vehicle_name):

        # connect to the AirSim simulator
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.vehicleName = vehicle_name
        self.echoName = echo_name
        self.lastEchoTimeStamp = 0

    def get_data(self):

        # get echo data
        echoData = self.client.getEchoData(self.echoName, self.vehicleName)

        if echoData.time_stamp != self.lastEchoTimeStamp:
            # Check if there are any points in the data
            if len(echoData.point_cloud) < 5:
                self.lastEchoTimeStamp = echoData.time_stamp
                return None
            else:
                self.lastEchoTimeStamp = echoData.time_stamp

                points = np.array(echoData.point_cloud, dtype=np.dtype('f4'))
                points = np.reshape(points, (int(points.shape[0] / 5), 5))
                points = points * np.array([1, -1, -1, 1, 1])
                print("got " + str(points.shape[0]) + " points")
                return points
        else:
            return None

    def stop(self):

        self.client.reset()
        print("Stopped!\n")


# main
if __name__ == "__main__":

    echoTest = EchoTest('echo2', 'airsimvehicle')
    points = echoTest.get_data()
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_box_aspect((np.ptp(points[:, 0]), np.ptp(points[:, 1]), np.ptp(points[:, 2])))
    ax.scatter(points[:, 0], points[:, 1], points[:, 2])
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()
