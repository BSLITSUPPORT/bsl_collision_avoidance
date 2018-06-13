#!/bin/bash
cd ~/bsl_ws
catkin_make
cd ~
source ~/bsl_ws/devel/setup.bash
roslaunch bsl_collision_avoidance collisionAvoidance.launch
