#!/bin/bash

SCRIPT_DIR="/home/unitree/catkin_ws/src/cv_camera/scripts_extra"

SCRIPT1="convert_depth_2_pc_front.py"
SCRIPT2="convert_depth_2_pc_back.py"

sleep 2

python ~/scripts_extra/convert_depth_2_pc_front.py &

python ~/scripts_extra/convert_depth_2_pc_back.py &

sleep 3

roslaunch slam_planner downsample_pcl.launch &

sleep 10

python ~/pcl_fusion/fusion_pcl.py &

wait
