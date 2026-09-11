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

"""
Integration tests for joint effort reporting.

Verifies that the driver can publish joint efforts from either RTDE field:
* ``actual_current_as_torque`` (default, ``use_currents_as_efforts:=false``)
* ``actual_current`` (legacy, ``use_currents_as_efforts:=true``)

Requires a UR5e URSim image with PolyScope >= 5.23.0 / 10.11.0 for the default torque mode.
"""

import math
import os
import sys
import time
import unittest

import launch_testing
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    ConfigurationInterface,
    DashboardInterface,
    IoStatusInterface,
    generate_driver_test_description,
)

# Approximate effort magnitude thresholds used to distinguish torques (Nm) from
# currents (A) at the default URSim pose. These are not exact and are only
# valid for a UR5e.
MIN_TORQUE_EFFORT_NM = 20.0
MIN_CURRENT_EFFORT_A = 2.0
UR_TYPE = "ur5e"


@pytest.mark.launch_test
@launch_testing.parametrize(
    "use_currents_as_efforts",
    # None: omit the launch argument and assert the default is torque mode.
    [(None), ("false"), ("true")],
)
def generate_test_description(use_currents_as_efforts):
    return generate_driver_test_description(
        ur_type=UR_TYPE,
        use_currents_as_efforts=use_currents_as_efforts,
    )


class JointEffortsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("joint_efforts_test")
        time.sleep(1)
        cls.init_robot(cls)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def init_robot(self):
        self._dashboard_interface = DashboardInterface(self.node)
        self._io_status_controller_interface = IoStatusInterface(self.node)
        self._configuration_controller_interface = ConfigurationInterface(self.node)

    def setUp(self):
        self._dashboard_interface.start_robot()
        time.sleep(1)
        self.assertTrue(self._io_status_controller_interface.resend_robot_program().success)

    def _wait_for_joint_state(self, timeout_sec=10.0):
        last_msg = None

        def cb(msg):
            nonlocal last_msg
            last_msg = msg

        sub = self.node.create_subscription(JointState, "/joint_states", cb, 10)
        deadline = time.monotonic() + timeout_sec
        while last_msg is None and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        self.assertIsNotNone(last_msg, "Timed out waiting for /joint_states")
        return last_msg

    def test_joint_efforts_are_published(self, use_currents_as_efforts):
        """Driver comes up and publishes finite joint efforts for the selected RTDE source."""
        # Default (argument omitted) and explicit false both select torque reporting.
        expect_torques = use_currents_as_efforts in (None, "false")

        if expect_torques:
            version = self._configuration_controller_interface.get_robot_software_version()
            # Mirror the version gate in URPositionHardwareInterface::on_configure
            unsupported = (
                (version.major == 5 and version.minor < 23)
                or (version.major == 10 and version.minor < 11)
                or version.major < 5
            )
            self.assertFalse(
                unsupported,
                f"URSim software {version.major}.{version.minor} does not support "
                "actual_current_as_torque; need >= 5.23.0 / 10.11.0 for the default effort mode.",
            )

        joint_state = self._wait_for_joint_state()

        self.assertEqual(len(joint_state.name), 6)
        self.assertEqual(
            len(joint_state.effort),
            6,
            "JointState.effort must contain one value per joint",
        )
        for name, effort in zip(joint_state.name, joint_state.effort):
            self.assertTrue(
                math.isfinite(effort),
                f"Effort for joint '{name}' is not finite: {effort}",
            )

        # At the default URSim pose, gravity-related joint torques (Nm) are much
        # larger than idle motor currents (A). Use that to verify the correct RTDE
        # source is selected.
        max_abs_effort = max(abs(e) for e in joint_state.effort)
        if expect_torques:
            self.assertGreater(
                max_abs_effort,
                MIN_TORQUE_EFFORT_NM,
                "Expected gravity-related joint torques > "
                f"{MIN_TORQUE_EFFORT_NM} Nm with use_currents_as_efforts="
                f"{use_currents_as_efforts!r}, but max |effort| was {max_abs_effort}. "
                "Check that actual_current_as_torque is read (default).",
            )
        else:
            self.assertGreater(
                max_abs_effort,
                MIN_CURRENT_EFFORT_A,
                "Expected motor currents > "
                f"{MIN_CURRENT_EFFORT_A} A with use_currents_as_efforts:=true, "
                f"but max |effort| was {max_abs_effort}. "
                "Check that actual_current is read.",
            )
            self.assertLess(
                max_abs_effort,
                MIN_TORQUE_EFFORT_NM,
                "Expected motor currents (A) to stay below typical gravity torque "
                f"magnitudes (Nm), but max |effort| was {max_abs_effort}.",
            )
