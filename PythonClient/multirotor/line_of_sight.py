import cosysairsim as airsim

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

home = client.getHomeGeoPoint()
print("home:\n%s" % home)

target = home
target.latitude -= 1

result = client.simTestLineOfSightToPoint(target)
print(f"test line of sight from vehicle to\n{target}\n\t:{result}")

result = client.simTestLineOfSightBetweenPoints(home, target)
print(f"test line of sight from home to\n{target}\n\t:{result}")

result = client.simGetWorldExtents()
print(f"world extents:\n{result[0]}\n\t-\n{result[1]}")

client.reset()
client.armDisarm(False)

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)
