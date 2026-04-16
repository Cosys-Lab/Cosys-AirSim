# from keras.models import load_model
import numpy as np

import cosysairsim as airsim

# if (len(sys.argv) != 2):
#    print('usage: python drive.py <modelName>')
#    sys.exit()

# print('Loading model...')
# model = load_model(sys.argv[1])

# connect to the AirSim simulator
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()

car_controls.steering = 0
car_controls.throttle = 0
car_controls.brake = 0

image_buf = np.zeros((1, 144, 256, 3))
state_buf = np.zeros((1, 4))


def get_image():
    image = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])[0]
    image1d = np.frombuffer(image.image_data_uint8.encode(), dtype=np.uint8)
    image_rgb = image1d.reshape(image.height, image.width, 3)
    return image_rgb


while True:
    car_state = client.getCarState()

    print(f"car speed: {car_state.speed}")

    if car_state.speed < 20:
        car_controls.throttle = 1.0
    else:
        car_controls.throttle = 0.0

    # state_buf[0] = np.array([car_controls.steering, car_controls.throttle, car_controls.brake, car_state.speed])
    # model_output = model.predict([image_buf, state_buf])
    # car_controls.steering = float(model_output[0][0])
    car_controls.steering = 0

    print(f"Sending steering = {car_controls.steering}, throttle = {car_controls.throttle}")

    client.setCarControls(car_controls)
