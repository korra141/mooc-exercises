#!/bin/bash


source /environment.sh

source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash --extend
source /code/submission_ws/devel/setup.bash --extend
source /code/solution/devel/setup.bash --extend

set -eux

dt-exec-BG roslaunch --wait agent agent_node.launch
dt-exec-BG roslaunch --wait car_interface all.launch veh:=$VEHICLE_NAME

# This runs the communication node and only the other needed nodes
dt-exec-FG roslaunch --wait duckietown_demos communication.launch || true


copy-ros-logs
