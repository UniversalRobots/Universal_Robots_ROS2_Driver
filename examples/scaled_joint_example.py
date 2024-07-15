#!/usr/bin/env python3
# Copyright 2024, Universal Robots
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from robot_class import Robot

if __name__ == "__main__":
    rclpy.init()
    node = Node("robot_driver_test")
    robot = Robot(node)

    # The following list are arbitrary joint positions, change according to your own needs
    waypts = [
        [-1.58, -1.692, -1.4311, -0.0174, 1.5882, 0.0349],
        [-3, -1.692, -1.4311, -0.0174, 1.5882, 0.0349],
        [-1.58, -1.692, -1.4311, -0.0174, 1.5882, 0.0349],
    ]
    time_vec = [Duration(sec=4, nanosec=0), Duration(sec=8, nanosec=0), Duration(sec=12, nanosec=0)]
    robot.switch_controllers(
        ["scaled_joint_trajectory_controller"], ["passthrough_trajectory_controller"]
    )
    # Execute trajectory on robot, make sure that the robot is booted and the control script is running
    robot.follow_trajectory(waypts, time_vec)
