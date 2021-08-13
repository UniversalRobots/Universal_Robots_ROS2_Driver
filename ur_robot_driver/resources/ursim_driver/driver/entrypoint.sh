#!/bin/bash

# Setup VNC server
rm /tmp/.X1-lock && rm /tmp/.X11-unix/X1
Xvfb :1 -screen 0 1366x768x16 &
x11vnc -rfbport 5566 -create -bg -quiet -forever -shared -display :1
/wait_dashboard_server.sh
source /~/workspace/ros_ws_foxy_ur_driver/install/setup.bash && ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.56.101 use_fake_hardware:=false launch_rviz:=false &
source /~/workspace/ros_ws_foxy_ur_driver/install/setup.bash && ros2 service call /dashboard_client/load_installation ur_dashboard_msgs/srv/Load "{filename: urcap_ros_control.installation}"
source /~/workspace/ros_ws_foxy_ur_driver/install/setup.bash && ros2 service call /dashboard_client/load_program ur_dashboard_msgs/srv/Load "{filename: urcap_ros_control.urp}"
source /~/workspace/ros_ws_foxy_ur_driver/install/setup.bash && ros2 service call /dashboard_client/power_on std_srvs/srv/Trigger
source /~/workspace/ros_ws_foxy_ur_driver/install/setup.bash && ros2 service call /dashboard_client/brake_release std_srvs/srv/Trigger
source /~/workspace/ros_ws_foxy_ur_driver/install/setup.bash && ros2 service call /dashboard_client/close_popup std_srvs/srv/Trigger
source /~/workspace/ros_ws_foxy_ur_driver/install/setup.bash && ros2 service call /dashboard_client/play  std_srvs/srv/Trigger
tail -F anything
