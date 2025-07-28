#!/usr/bin/env python3
# Copyright 2025, Universal Robots A/S
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
from rclpy.executors import ExternalShutdownException

from scipy.spatial.transform import Rotation as R, Slerp

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from examples import Robot


class PDTorqueControllerExample(Node):
    JOINT_START = [-1.6, -1.55, -1.7, -1.0, 2.05, 0.5]
    JOINT_TARGET = [-0.6, -1.15, -1.0, 1.0, 0.05, -0.5]
    TASK_START = PoseStamped()
    TASK_END = PoseStamped()

    INTERPOLATION_STEPS = 100
    timer_period = 0.02  # 50 Hz

    def __init__(self):
        super().__init__("pd_torque_example")

        self.robot = Robot(self)
        self.joint_target_pub = self.create_publisher(
            Float64MultiArray, "/pd_torque_controller/joint_commands", 1
        )
        self.task_space_target_pub = self.create_publisher(
            PoseStamped, "/pd_torque_controller/task_space_commands", 1
        )

        self.TASK_START.header.frame_id = "base"
        self.TASK_START.pose.position.x = -0.10223
        self.TASK_START.pose.position.y = -0.50695
        self.TASK_START.pose.position.z = 0.51035
        self.TASK_START.pose.orientation.x = 0.194808
        self.TASK_START.pose.orientation.y = 0.980198
        self.TASK_START.pose.orientation.z = -0.162579
        self.TASK_START.pose.orientation.w = 0.276547

        self.TASK_END.header.frame_id = "base"
        self.TASK_END.pose.position.x = 0.2
        self.TASK_END.pose.position.y = -0.2
        self.TASK_END.pose.position.z = 0.2
        self.TASK_END.pose.position.z = 0.2
        self.TASK_END.pose.orientation.x = 0.519404
        self.TASK_END.pose.orientation.y = -0.479812
        self.TASK_END.pose.orientation.z = 0.533193
        self.TASK_END.pose.orientation.w = -0.464441

        self._step = 0
        self.timer = None

        self.robot.init_robot()
        self.startup()
        time.sleep(0.5)
        self.move_joint_based()
        time.sleep(0.5)
        self.move_to_starting_pose()
        time.sleep(0.5)
        self.move_task_space()

    def move_to_starting_pose(self):
        self.robot.call_service(
            "/controller_manager/switch_controller",
            SwitchController.Request(
                deactivate_controllers=[
                    "force_mode_controller",
                    "forward_effort_controller",
                    "forward_position_controller",
                    "forward_velocity_controller",
                    "freedrive_mode_controller",
                    "joint_trajectory_controller",
                    "pd_torque_controller",
                    "scaled_joint_trajectory_controller",
                    "tool_contact_controller",
                ],
                activate_controllers=[
                    "passthrough_trajectory_controller",
                ],
                strictness=SwitchController.Request.BEST_EFFORT,
            ),
        )
        # Move robot in to position
        self.robot.send_trajectory(
            waypts=[self.JOINT_START],
            time_vec=[Duration(sec=1, nanosec=0)],
            action_client=self.robot.passthrough_trajectory_action_client,
        )

    def move_joint_based(self):
        # Switch to pd_torque_controller
        self.robot.call_service(
            "/controller_manager/switch_controller",
            SwitchController.Request(
                deactivate_controllers=[
                    "force_mode_controller",
                    "forward_effort_controller",
                    "forward_position_controller",
                    "forward_velocity_controller",
                    "freedrive_mode_controller",
                    "joint_trajectory_controller",
                    "passthrough_trajectory_controller",
                    "scaled_joint_trajectory_controller",
                    "tool_contact_controller",
                ],
                activate_controllers=["pd_torque_controller"],
                strictness=SwitchController.Request.BEST_EFFORT,
            ),
        )
        self._step = 0
        self.get_logger().info("Starting joint-based movement")
        self.timer = self.create_timer(self.timer_period, self.on_timer_joint, autostart=True)

    def move_task_space(self):
        # Switch to pd_torque_controller
        self.robot.call_service(
            "/controller_manager/switch_controller",
            SwitchController.Request(
                deactivate_controllers=[
                    "force_mode_controller",
                    "forward_effort_controller",
                    "forward_position_controller",
                    "forward_velocity_controller",
                    "freedrive_mode_controller",
                    "joint_trajectory_controller",
                    "passthrough_trajectory_controller",
                    "scaled_joint_trajectory_controller",
                    "tool_contact_controller",
                ],
                activate_controllers=["pd_torque_controller"],
                strictness=SwitchController.Request.BEST_EFFORT,
            ),
        )
        self._step = 0
        self.get_logger().info("Starting task space movement")
        self.timer = self.create_timer(self.timer_period, self.on_timer_task, autostart=True)

    def startup(self):
        # Press play on the robot
        self.robot.call_service("/dashboard_client/play", Trigger.Request())

        time.sleep(0.5)
        # Start controllers
        self.move_to_starting_pose()

    def on_timer_joint(self):
        # Publish joint commands
        joint_commands = Float64MultiArray()
        joint_commands.data = [
            a + (b - a) * self._step / self.INTERPOLATION_STEPS
            for a, b in zip(self.JOINT_START, self.JOINT_TARGET)
        ]
        self.joint_target_pub.publish(joint_commands)

        self.get_logger().info(
            f"Step {self._step}/{self.INTERPOLATION_STEPS}, "
            f"Joint Commands: {joint_commands.data}"
        )

        if self._step >= self.INTERPOLATION_STEPS:
            self.timer.cancel()
        self._step += 1

    def on_timer_task(self):
        # Publish task space commands
        task_space_command = PoseStamped()
        task_space_command.header.frame_id = "base"
        task_space_command.pose.position.x = (
            self.TASK_START.pose.position.x
            + (self.TASK_END.pose.position.x - self.TASK_START.pose.position.x)
            * self._step
            / self.INTERPOLATION_STEPS
        )
        task_space_command.pose.position.y = (
            self.TASK_START.pose.position.y
            + (self.TASK_END.pose.position.y - self.TASK_START.pose.position.y)
            * self._step
            / self.INTERPOLATION_STEPS
        )
        task_space_command.pose.position.z = (
            self.TASK_START.pose.position.z
            + (self.TASK_END.pose.position.z - self.TASK_START.pose.position.z)
            * self._step
            / self.INTERPOLATION_STEPS
        )

        # If someone knows whether this is possible with pure rclpy, please let me know.
        key_times = [0, self.INTERPOLATION_STEPS]
        rotations = R.from_quat(
            [
                [
                    self.TASK_END.pose.orientation.x,
                    self.TASK_END.pose.orientation.y,
                    self.TASK_END.pose.orientation.z,
                    self.TASK_END.pose.orientation.w,
                ],
                [
                    self.TASK_START.pose.orientation.x,
                    self.TASK_START.pose.orientation.y,
                    self.TASK_START.pose.orientation.z,
                    self.TASK_START.pose.orientation.w,
                ],
            ]
        )
        slerp = Slerp(key_times, rotations)
        interp_rot = slerp(self._step)
        interp_quat = interp_rot.as_quat()

        task_space_command.pose.orientation.x = interp_quat[0]
        task_space_command.pose.orientation.y = interp_quat[1]
        task_space_command.pose.orientation.z = interp_quat[2]
        task_space_command.pose.orientation.w = interp_quat[3]

        self.task_space_target_pub.publish(task_space_command)

        if self._step >= self.INTERPOLATION_STEPS:
            self.timer.cancel()
        self._step += 1


if __name__ == "__main__":
    rclpy.init()

    node = PDTorqueControllerExample()

    node.get_logger().info("PDTorqueControllerExample started.")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
