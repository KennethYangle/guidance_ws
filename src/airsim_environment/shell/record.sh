#!/bin/bash

cp ~/guidance_ws/src/airsim_environment/settings/settings_ros.json ~/Documents/AirSim/settings.json
cd /home/zhenglong/AirsimEnvironment/Neighborhood
./AirSimNH.sh -opengl4 & PID0=$!
sleep 15s
roslaunch airsim_ros_pkgs airsim_node.launch & PID1=$!
sleep 3s
rosservice call /airsim_node/Drone0/takeoff "waitOnLastTask: true" & PID2=$!
sleep 7s
rosservice call /airsim_node/Drone0/takeoff "waitOnLastTask: true" & PID3=$!
sleep 7s
roslaunch airsim_environment 3.launch & PID4=$!

wait
kill PID4 PID3 PID2 PID1 PID0
exit