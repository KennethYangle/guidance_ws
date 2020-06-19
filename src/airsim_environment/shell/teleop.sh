#!/bin/bash

Environment_names=("building" "neighborhood" "mountain" "soccer")
path=$(cd `dirname $0`;pwd)

if [ ! -n "$1" ]
then
    echo "Usage: ./teleop.sh Environment_name(${Environment_names[@]})"
    exit 1
fi

if [[ ! "${Environment_names[@]}" =~ $1 ]]
then
    echo "Usage: ./teleop.sh Environment_name(${Environment_names[@]})"
    exit 1
fi

cp ~/guidance_ws/src/airsim_environment/settings/settings_$1_record.json ~/Documents/AirSim/settings.json

if [ $1 == ${Environment_names[0]} ]
then
    cd /home/zhenglong/AirsimEnvironment/Building_99
    ./Building_99.sh -opengl4 & PID0=$!
elif [ $1 == ${Environment_names[1]} ]
then
    cd /home/zhenglong/AirsimEnvironment/Neighborhood
    ./AirSimNH.sh -opengl4 & PID0=$!
elif [ $1 == ${Environment_names[2]} ]
then
    cd /home/zhenglong/AirsimEnvironment/LandscapeMountains
    ./LandscapeMountains.sh -opengl4 & PID0=$!
elif [ $1 == ${Environment_names[3]} ]
then
    cd /home/zhenglong/AirsimEnvironment/SoccerField
    ./SoccerField.sh -opengl4 & PID0=$!
else :
fi

cd "${path}/../bag/"
if [ ! -d "$1" ]; then
    mkdir "$1"
fi

sleep 15s
roslaunch airsim_environment 1.launch & PID1=$!
sleep 3s
rosservice call /airsim_node/Drone0/takeoff "waitOnLastTask: true" & PID2=$!
sleep 7s
rosservice call /airsim_node/Drone0/takeoff "waitOnLastTask: true" & PID3=$!
sleep 7s
roslaunch airsim_environment 2.launch folder:="$1" & PID4=$!

wait
kill PID4 PID3 PID2 PID1 PID0
exit