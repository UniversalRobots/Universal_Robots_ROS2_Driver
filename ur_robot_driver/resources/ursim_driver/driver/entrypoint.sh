#!/bin/bash

# Setup VNC server
rm /tmp/.X1-lock && rm /tmp/.X11-unix/X1
Xvfb :1 -screen 0 1366x768x16 &
x11vnc -rfbport 5566 -create -bg -quiet -forever -shared -display :1
source /~/workspace/ros_ws_foxy_ur_driver/install/setup.bash && ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.56.101 use_fake_hardware:=false launch_rviz:=true
