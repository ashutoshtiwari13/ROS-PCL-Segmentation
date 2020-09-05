#! /bin/bash
x-terminal-emulator -e roslaunch sensor_stick robot_spawn.launch &
sleep 3 &&
rosrun sensor_stick segmentation.py
