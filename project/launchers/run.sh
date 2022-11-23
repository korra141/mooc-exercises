#!/bin/bash


source /environment.sh

source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash --extend
source /code/submission_ws/devel/setup.bash --extend
source /code/solution/devel/setup.bash --extend

set -eux

dt-exec-BG roslaunch --wait agent agent_node.launch
dt-exec-BG roslaunch --wait car_interface all.launch veh:=$VEHICLE_NAME

# Will activate this once we run the full demo
#dt-exec-FG roslaunch --wait duckietown_demos communication.launch || true
# For the moment let's just run our specific node since we dont need any other one for testing
dt-exec-FG roslaunch --wait communication communication_node_standalone_test.launch || true

copy-ros-logs
