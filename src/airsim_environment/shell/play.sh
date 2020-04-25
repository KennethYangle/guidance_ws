#!/bin/bash

roslaunch airsim_environment rosbag_play.launch & PID0=$!

wait
kill PID0
exit