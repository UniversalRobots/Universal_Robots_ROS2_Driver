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

import pytest

import launch_testing
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Inertia, Vector3
from ur_msgs.srv import SetPayload
from builtin_interfaces.msg import Duration

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    ControllerManagerInterface,
    DashboardInterface,
    IoStatusInterface,
    generate_driver_test_description,
)


@pytest.mark.launch_test
@launch_testing.parametrize(
    "tf_prefix",
    [(""), ("my_ur_")],
)
def generate_test_description(tf_prefix):
    return generate_driver_test_description(tf_prefix=tf_prefix)


def _make_inertia(
    m=0.0, cx=0.0, cy=0.0, cz=0.0, ixx=0.0, ixy=0.0, ixz=0.0, iyy=0.0, iyz=0.0, izz=0.0
):
    """Helper to build a geometry_msgs/Inertia message."""

    msg = Inertia()
    msg.m = m
    msg.com = Vector3(x=cx, y=cy, z=cz)
    msg.ixx = ixx
    msg.ixy = ixy
    msg.ixz = ixz
    msg.iyy = iyy
    msg.iyz = iyz
    msg.izz = izz
    return msg


class RobotDriverTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = Node("robot_driver_test")
        time.sleep(1)
        cls.init_robot(cls)

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        cls.node.destroy_node()
        rclpy.shutdown()

    def init_robot(self):
        self._dashboard_interface = DashboardInterface(self.node)
        self._controller_manager_interface = ControllerManagerInterface(self.node)
        self._io_status_controller_interface = IoStatusInterface(self.node)
        self._set_payload_client = self.node.create_client(
            SetPayload, "/io_and_status_controller/set_payload"
        )

    def setUp(self):
        self._dashboard_interface.start_robot()
        time.sleep(1)
        self.assertTrue(self._io_status_controller_interface.resend_robot_program().success)
        # Reset payload to a known zero state before each test
        self._call_set_payload(_make_inertia())

    def _call_set_payload(self, inertia_msg, transition_time=0.0):
        """Call the set_payload service and return the response."""

        req = SetPayload.Request()
        req.payload = inertia_msg
        sec = int(transition_time)
        nanosec = int((transition_time - sec) * 1e9)
        req.transition_time = Duration(sec=sec, nanosec=nanosec)
        future = self._set_payload_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is None:
            raise Exception(f"set_payload service call failed: {future.exception()}")
        return future.result()

    def test_set_payload_with_inertia(self, tf_prefix):
        """Setting mass, COG and full inertia matrix should succeed."""

        res = self._call_set_payload(
            _make_inertia(
                m=1.0,
                cx=0.1,
                cy=0.0,
                cz=0.2,
                ixx=0.01,
                iyy=0.01,
                izz=0.02,
                ixy=0.0,
                ixz=0.0,
                iyz=0.0,
            ),
            transition_time=0.0,
        )
        self.assertTrue(res.success)

    def test_set_payload_with_transition_time(self, tf_prefix):
        """Setting payload with transition_time > 0 should succeed.
        The service should wait for the transition to complete before verifying."""

        res = self._call_set_payload(
            _make_inertia(
                m=1.0,
                cx=0.0,
                cy=0.0,
                cz=0.1,
                ixx=0.01,
                iyy=0.01,
                izz=0.02,
            ),
            transition_time=1.0,
        )
        self.assertTrue(res.success)

    def test_set_payload_updates_sequentially(self, tf_prefix):
        """Multiple sequential set_payload calls should all succeed."""
        payloads = [
            _make_inertia(m=0.5, cx=0.0, cy=0.0, cz=0.1, ixx=0.005, iyy=0.005, izz=0.01),
            _make_inertia(m=1.0, cx=0.1, cy=0.0, cz=0.2, ixx=0.01, iyy=0.01, izz=0.02),
            _make_inertia(m=2.0, cx=0.05, cy=0.05, cz=0.15, ixx=0.02, iyy=0.02, izz=0.04),
            _make_inertia(m=0.0, cx=0.0, cy=0.0, cz=0.0, ixx=0.0, iyy=0.0, izz=0.0),
        ]
        for payload in payloads:
            res = self._call_set_payload(payload, transition_time=0.0)
            self.assertTrue(res.success, f"set_payload failed for m={payload.m}")
