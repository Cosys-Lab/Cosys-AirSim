import setup_path
import cosysairsim as airsim
import random
import time

client = airsim.VehicleClient()
client.confirmConnection()

assets = client.simListAssets()
print(f"Assets: {assets}")

scale = airsim.Vector3r(1.0, 1.0, 1.0)

# asset_name = random.choice(assets)
asset_name = '1M_Cube_Chamfer'

desired_name = f"{asset_name}_spawn_{random.randint(0, 100)}"
pose = airsim.Pose(position_val=airsim.Vector3r(5.0, 0.0, 0.0))

obj_name = client.simSpawnObject(desired_name, asset_name, pose, scale, True)

print(f"Created object {obj_name} from asset {asset_name} "
      f"at pose {pose}, scale {scale}")

# Blueprints can be spawned as well by setting is_blueprint to True.
# In a packaged build the blueprint has to be cooked, see the API documentation.
blueprint_name = 'BP_PointLight'
light_name = client.simSpawnObject(f"{blueprint_name}_spawn", blueprint_name,
                                   airsim.Pose(position_val=airsim.Vector3r(5.0, 0.0, -2.0)), scale, False, True)
print(f"Created object {light_name} from blueprint {blueprint_name}")

all_objects = client.simListSceneObjects()
for name in [obj_name, light_name]:
    if name not in all_objects:
        print(f"Object {name} not present!")

time.sleep(10.0)

for name in [obj_name, light_name]:
    print(f"Destroying {name}")
    client.simDestroyObject(name)
