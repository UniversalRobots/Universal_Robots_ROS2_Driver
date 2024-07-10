#!/usr/bin/env python3
# Copyright 2024, Universal Robots A/S
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

import time

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from geometry_msgs.msg import (
    Point,
    Quaternion,
    Pose,
    PoseStamped,
    Wrench,
    WrenchStamped,
    Vector3,
)

from ur_msgs.srv import SetForceMode

from examples import Robot

if __name__ == "__main__":
    rclpy.init()
    node = Node("robot_driver_test")
    robot = Robot(node)

    # Activate force mode controller
    robot.call_service(
        "/controller_manager/switch_controller",
        SwitchController.Request(
            activate_controllers=["force_mode_controller", "scaled_joint_trajectory_controller"],
            strictness=SwitchController.Request.BEST_EFFORT,
        ),
    )

    # Add force mode service to service interfaces and re-init robot
    robot.service_interfaces.update({"/force_mode_controller/start_force_mode": SetForceMode})
    robot.init_robot()
    time.sleep(2)
    # Press play on the robot
    robot.call_service("/dashboard_client/play", Trigger.Request())

    # Move robot in to position
    robot.send_trajectory(
        waypts=[[-1.5707, -1.5707, -1.5707, -1.5707, 1.5707, 0]],
        time_vec=[Duration(sec=6, nanosec=0)],
    )

    # Finished moving

    # Create task frame for force mode
    point = Point(x=0.0, y=0.0, z=0.0)
    orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
    task_frame_pose = Pose()
    task_frame_pose.position = point
    task_frame_pose.orientation = orientation
    header = Header(seq=1, frame_id="world")
    header.stamp.sec = int(time.time()) + 1
    header.stamp.nanosec = 0
    frame_stamp = PoseStamped()
    frame_stamp.header = header
    frame_stamp.pose = task_frame_pose

    # Create compliance vector (which axes should be force controlled)
    compliance = [False, False, True, False, False, False]

    # Create Wrench message for force mode
    wrench_vec = Wrench(force=Vector3(x=0.0, y=0.0, z=-10.0), torque=Vector3(x=0.0, y=0.0, z=0.0))
    wrench_stamp = WrenchStamped(header=header, wrench=wrench_vec)
    # Specify interpretation of task frame (no transform)
    type_spec = SetForceMode.Request.NO_TRANSFORM

    # Specify max speeds and deviations of force mode
    limits = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

    # specify damping and gain scaling
    damping_factor = 0.025
    gain_scale = 0.5

    req = SetForceMode.Request()
    req.task_frame = frame_stamp
    req.selection_vector_x = compliance[0]
    req.selection_vector_y = compliance[1]
    req.selection_vector_z = compliance[2]
    req.selection_vector_rx = compliance[3]
    req.selection_vector_ry = compliance[4]
    req.selection_vector_rz = compliance[5]
    req.wrench = wrench_stamp
    req.type = type_spec
    req.limits = limits
    req.damping_factor = damping_factor
    req.gain_scaling = gain_scale

    # Send request to controller
    robot.call_service("/force_mode_controller/start_force_mode", req)

    time.sleep(15)
    # Deactivate force mode controller
    robot.call_service(
        "/controller_manager/switch_controller",
        SwitchController.Request(
            deactivate_controllers=["force_mode_controller"],
            strictness=SwitchController.Request.BEST_EFFORT,
        ),
    )
