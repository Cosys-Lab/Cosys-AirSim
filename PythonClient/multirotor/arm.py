import cosysairsim as airsim

client = airsim.MultirotorClient()
client.confirmConnection()
client.armDisarm(True)
