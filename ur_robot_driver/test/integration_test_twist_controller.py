#!/usr/bin/env python3
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

import math
import os
import sys
import time
import unittest

import pytest

import launch_testing
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from controller_manager_msgs.srv import SwitchController

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    ControllerManagerInterface,
    DashboardInterface,
    IoStatusInterface,
    generate_driver_test_description,
)

MIN_TCP_DISPLACEMENT = 0.005


@pytest.mark.launch_test
@launch_testing.parametrize("tf_prefix", [(""), ("my_ur_")])
def generate_test_description(tf_prefix):
    return generate_driver_test_description(tf_prefix=tf_prefix)


class TwistControllerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("twist_controller_test")
        time.sleep(1)
        cls.init_robot(cls)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def init_robot(self):
        self._dashboard_interface = DashboardInterface(self.node)
        self._controller_manager_interface = ControllerManagerInterface(self.node)
        self._io_status_controller_interface = IoStatusInterface(self.node)
        self._controller_manager_interface.wait_for_controller("twist_controller")
        self._dashboard_interface.start_robot()

    def setUp(self):
        time.sleep(1)
        self.assertTrue(self._io_status_controller_interface.resend_robot_program().success)

    def _base_frame(self, tf_prefix):
        return tf_prefix + "base"

    def _wait_for_tcp_pose(self, timeout=5.0):
        pose = PoseStamped()
        received = {"value": False}

        def callback(msg):
            pose.pose = msg.pose
            received["value"] = True

        subscription = self.node.create_subscription(
            PoseStamped,
            "/tcp_pose_broadcaster/pose",
            callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )

        start_time = time.time()
        while not received["value"] and time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_subscription(subscription)
        self.assertTrue(received["value"], "Did not receive TCP pose from tcp_pose_broadcaster")
        return pose

    def _activate_twist_controller(self):
        return self._controller_manager_interface.switch_controller(
            strictness=SwitchController.Request.BEST_EFFORT,
            activate_controllers=["twist_controller"],
            deactivate_controllers=[
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
                "passthrough_trajectory_controller",
            ],
        ).ok

    def _deactivate_twist_controller(self):
        return self._controller_manager_interface.switch_controller(
            strictness=SwitchController.Request.BEST_EFFORT,
            deactivate_controllers=["twist_controller"],
        ).ok

    def test_start_twist_controller(self):
        self.assertTrue(self._activate_twist_controller())
        self.assertTrue(self._deactivate_twist_controller())

    def test_cartesian_velocity_motion(self, tf_prefix):
        self.assertTrue(self._activate_twist_controller())

        initial_pose = self._wait_for_tcp_pose()
        initial_z = initial_pose.pose.position.z

        publisher = self.node.create_publisher(TwistStamped, "/twist_controller/twist", 10)
        base_frame = self._base_frame(tf_prefix)
        speed = 0.1
        time_in_motion = 1.0

        cmd = TwistStamped()
        cmd.header.frame_id = base_frame
        cmd.twist.linear.z = speed

        start_time = time.time()
        publish_period = 0.1
        while time.time() - start_time < time_in_motion:
            cmd.header.stamp = self.node.get_clock().now().to_msg()
            publisher.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=publish_period)

        cmd.twist.linear.z = 0.0
        for _ in range(10):
            cmd.header.stamp = self.node.get_clock().now().to_msg()
            publisher.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=publish_period)

        final_pose = self._wait_for_tcp_pose()
        displacement = final_pose.pose.position.z - initial_z
        expected_displacement = speed * time_in_motion

        self.node.destroy_publisher(publisher)
        self.assertTrue(self._deactivate_twist_controller())

        self.assertAlmostEqual(displacement, expected_displacement, delta=0.05)

    def test_wrong_frame_is_ignored(self, tf_prefix):
        self.assertTrue(self._activate_twist_controller())

        initial_pose = self._wait_for_tcp_pose()
        initial_position = (
            initial_pose.pose.position.x,
            initial_pose.pose.position.y,
            initial_pose.pose.position.z,
        )

        publisher = self.node.create_publisher(TwistStamped, "/twist_controller/twist", 10)

        cmd = TwistStamped()
        cmd.header.frame_id = "tool0"
        cmd.twist.linear.x = 0.05

        start_time = time.time()
        publish_period = 0.1
        while time.time() - start_time < 0.5:
            cmd.header.stamp = self.node.get_clock().now().to_msg()
            publisher.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=publish_period)

        final_pose = self._wait_for_tcp_pose()
        final_position = (
            final_pose.pose.position.x,
            final_pose.pose.position.y,
            final_pose.pose.position.z,
        )

        self.node.destroy_publisher(publisher)
        self.assertTrue(self._deactivate_twist_controller())

        displacement = math.sqrt(
            sum((a - b) ** 2 for a, b in zip(initial_position, final_position))
        )
        self.assertLess(
            displacement,
            MIN_TCP_DISPLACEMENT,
            "TCP moved despite publishing twist in unsupported frame",
        )

    def test_twist_and_tool_contact_are_compatible(self):
        self.assertTrue(self._activate_twist_controller())
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                activate_controllers=["tool_contact_controller"],
                deactivate_controllers=[],
            ).ok
        )
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                deactivate_controllers=["twist_controller", "tool_contact_controller"],
            ).ok
        )
