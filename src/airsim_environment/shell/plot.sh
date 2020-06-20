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

rosrun plotjuggler PlotJuggler -d ~/guidance_ws/src/airsim_environment/bag/$1/data_phase.bag -l ~/guidance_ws/src/airsim_environment/juggler/plot.xml