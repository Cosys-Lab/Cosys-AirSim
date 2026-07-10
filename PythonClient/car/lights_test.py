import cosysairsim as airsim
import time

client = airsim.CarClient()
client.confirmConnection()

client.simSetWorldLightIntensity("worldlight1", 4)
time.sleep(5)
client.simSetWorldLightIntensity("worldlight1", 16)
time.sleep(5)
client.simSetWorldLightVisibility("worldlight1", False)
time.sleep(5)
client.simSetWorldLightVisibility("worldlight1", True)

time.sleep(5)

client.simSetVehicleLightIntensity("airsimvehicle", "vehiclelight1", 4)
time.sleep(5)
client.simSetVehicleLightIntensity("airsimvehicle", "vehiclelight1", 16)
time.sleep(5)
client.simSetVehicleLightVisibility("airsimvehicle", "vehiclelight1", False)
time.sleep(5)
client.simSetVehicleLightVisibility("airsimvehicle", "vehiclelight1", True)

print("done")
