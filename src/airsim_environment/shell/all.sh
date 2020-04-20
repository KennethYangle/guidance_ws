#!/bin/bash

cd /home/zhenglong/AirsimEnvironment/Neighborhood
./AirSimNH.sh -opengl4 &
sleep 15
roslaunch airsim_environment all.launch