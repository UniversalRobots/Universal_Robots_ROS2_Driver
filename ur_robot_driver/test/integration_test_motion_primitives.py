#!/usr/bin/env python
# Copyright 2026, Universal Robots A/S
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

import os
import sys
import time
import unittest

import launch_testing
import pytest
import rclpy
from control_msgs.action import ExecuteMotionPrimitiveSequence
from control_msgs.msg import MotionArgument, MotionPrimitive, MotionPrimitiveSequence
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    ActionInterface,
    ControllerManagerInterface,
    DashboardInterface,
    IoStatusInterface,
    generate_driver_test_description,
    wait_for_robot_program_state,
)

MOTION_PRIMITIVE_CONTROLLER = "motion_primitive_forward_controller"
ALL_MOTION_CONTROLLERS = [
    "joint_trajectory_controller",
    "forward_position_controller",
    "forward_velocity_controller",
    "passthrough_trajectory_controller",
    "force_mode_controller",
    "freedrive_mode_controller",
    MOTION_PRIMITIVE_CONTROLLER,
    "twist_controller",
]
TIMEOUT_EXECUTE_MOTION_PRIMITIVE = 60


@pytest.mark.launch_test
@launch_testing.parametrize("tf_prefix", [(""), ("my_ur_")])
def generate_test_description(tf_prefix):
    return generate_driver_test_description(tf_prefix=tf_prefix)


def make_motion_argument(name, value):
    return MotionArgument(name=name, value=value)


def make_pose(x, y, z, qx=1.0, qy=0.0, qz=0.0, qw=0.0):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def make_primitive(
    motion_type, joint_positions=None, poses=None, move_time=2.0, vel=0.0, acc=0.0, blend_radius=0.0
):
    primitive = MotionPrimitive()
    primitive.type = motion_type
    primitive.blend_radius = blend_radius
    primitive.additional_arguments = [
        make_motion_argument("move_time", move_time),
        make_motion_argument("velocity", vel),
        make_motion_argument("acceleration", acc),
    ]

    if joint_positions is not None:
        primitive.joint_positions = joint_positions
    if poses is not None:
        primitive.poses = poses

    return primitive


class MotionPrimitivesControllerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("motion_primitives_controller_test")
        time.sleep(1)
        cls.init_robot()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def init_robot(cls):
        cls._dashboard_interface = DashboardInterface(cls.node)
        cls._controller_manager_interface = ControllerManagerInterface(cls.node)
        cls._io_status_controller_interface = IoStatusInterface(cls.node)
        cls._motion_sequence = ActionInterface(
            cls.node,
            f"/{MOTION_PRIMITIVE_CONTROLLER}/motion_sequence",
            ExecuteMotionPrimitiveSequence,
        )

        for controller in ALL_MOTION_CONTROLLERS:
            cls._controller_manager_interface.wait_for_controller(controller)

    def setUp(self):
        self._dashboard_interface.start_robot()
        time.sleep(1)
        resend_robot_program = getattr(self._io_status_controller_interface, "resend_robot_program")
        self.assertTrue(resend_robot_program().success)
        program_state = wait_for_robot_program_state(self.node, True, 10.0)
        self.assertEqual(program_state, True)
        switch_controller = getattr(self._controller_manager_interface, "switch_controller")
        self.assertTrue(
            switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                deactivate_controllers=ALL_MOTION_CONTROLLERS,
            ).ok
        )
        self.assertTrue(
            switch_controller(
                strictness=SwitchController.Request.STRICT,
                activate_controllers=[MOTION_PRIMITIVE_CONTROLLER],
            ).ok
        )

    def execute_motion_sequence(self, motions):
        goal_handle = self._motion_sequence.send_goal(
            trajectory=MotionPrimitiveSequence(motions=motions)
        )
        assert goal_handle is not None
        self.assertTrue(goal_handle.accepted)
        result = self._motion_sequence.get_result(goal_handle, TIMEOUT_EXECUTE_MOTION_PRIMITIVE)
        self.assertEqual(result.error_code, ExecuteMotionPrimitiveSequence.Result.SUCCESSFUL)

    def test_linear_joint_accepts_pose_target(self):
        self.execute_motion_sequence(
            [
                make_primitive(
                    MotionPrimitive.LINEAR_JOINT,
                    poses=[make_pose(0.174, -0.3, 0.3)],
                )
            ]
        )

    def test_linear_cartesian_accepts_joint_target(self):
        self.execute_motion_sequence(
            [
                make_primitive(
                    MotionPrimitive.LINEAR_CARTESIAN,
                    joint_positions=[0.9, -1.57, 1.57, -1.57, -1.57, -1.57],
                ),
            ]
        )

    def test_circular_cartesian_uses_goal_and_via_pose(self):
        self.execute_motion_sequence(
            [
                make_primitive(
                    MotionPrimitive.CIRCULAR_CARTESIAN,
                    poses=[
                        make_pose(0.3, -0.3, 0.1),
                        make_pose(0.174, -0.4, 0.1),
                    ],
                    vel=0.5,
                    acc=0.5,
                ),
            ]
        )

    def test_ambiguous_joint_and_pose_target_is_rejected(self):
        goal_handle = self._motion_sequence.send_goal(
            trajectory=MotionPrimitiveSequence(
                motions=[
                    make_primitive(
                        MotionPrimitive.LINEAR_JOINT,
                        joint_positions=[1.57, -1.57, 1.57, -1.57, -1.57, -1.57],
                        poses=[make_pose(0.174, -0.3, 0.3)],
                    )
                ]
            )
        )

        assert goal_handle is not None
        self.assertFalse(goal_handle.accepted)
