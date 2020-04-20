#!/bin/bash

cd /home/zhenglong/AirsimEnvironment/Neighborhood
./AirSimNH.sh -opengl4 &
sleep 15s
python3 /home/zhenglong/guidance_ws/src/airsim_environment/src/schematic.py