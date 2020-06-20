#!/bin/bash

Environment_names=("building" "neighborhood" "mountain" "soccer")
path=$(cd `dirname $0`;pwd)

if [ ! -n "$1" ]
then
    echo "Usage: ./play.sh Environment_name(${Environment_names[@]})"
    exit 1
fi

if [[ ! "${Environment_names[@]}" =~ $1 ]]
then
    echo "Usage: ./play.sh Environment_name(${Environment_names[@]})"
    exit 1
fi

roslaunch airsim_environment rosbag_play.launch folder:="$1" & PID0=$!

wait
kill PID0
exit