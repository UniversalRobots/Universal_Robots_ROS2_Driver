#!/bin/bash

# Setup VNC server
Xvfb :1 -screen 0 800x600x16 &
x11vnc -bg -quiet -forever -shared -display :1

# Execute URSim
/ursim/start-ursim.sh ${ROBOT_MODEL}