#!/bin/bash
sed -i "s/agent/${VEHICLE_NAME}/g" /opt/ros/noetic/share/rviz/odometry.rviz
rviz -d /opt/ros/noetic/share/rviz/odometry.rviz &
python3 /root/Documents/PID-heading-control.py &
rostopic pub /$VEHICLE_NAME/activity_name std_msgs/String "data: pid_heading"
