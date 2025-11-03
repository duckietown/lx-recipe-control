#!/bin/bash

source /environment.sh

source /opt/ros/noetic/setup.bash
source /code/devel/setup.bash --extend

exec roslaunch pid_controller pid_controller_node.launch veh:=$VEHICLE_NAME