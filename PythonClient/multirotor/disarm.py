import setup_path
import cosysairsim as airsim

client = airsim.MultirotorClient()
client.armDisarm(False)
