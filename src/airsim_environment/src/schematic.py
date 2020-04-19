import airsim

import numpy as np
import os
import tempfile
import pprint

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

client.takeoffAsync().join()
client.moveByVelocityAsync(1, 0, 0, 60).join()

client.armDisarm(False)
client.reset()
client.enableApiControl(False)