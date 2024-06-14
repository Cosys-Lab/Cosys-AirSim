import setup_path
import cosysairsim

client = airsim.MultirotorClient()
client.confirmConnection()
client.armDisarm(True)
