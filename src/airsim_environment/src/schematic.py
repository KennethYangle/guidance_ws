#!/usr/bin/env python3
#coding=utf-8

import airsim
import numpy as np

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

client.takeoffAsync().join()
# # line
# client.moveByVelocityAsync(1, 0, 0, 60).join()

# circle
while True:
    kinematics = client.simGetGroundTruthKinematics()
    pitch, roll, yaw = airsim.to_eularian_angles(kinematics.orientation)
    client.moveByVelocityAsync(np.cos(yaw), np.sin(yaw), -0.2, 1, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(True, 0.03*180/np.pi))

client.armDisarm(False)
client.reset()
client.enableApiControl(False)