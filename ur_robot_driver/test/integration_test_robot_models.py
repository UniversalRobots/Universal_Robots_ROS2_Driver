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
Integration test for robot model verification.

Integration test that brings up URSim for every supported robot model and verifies
that the driver starts and the hardware interface ends up in the active state.

The ``ursim_type`` / ``driver_type`` parameters are sourced from
:func:`launch_testing.parametrize`, so a fresh URSim container plus a fresh driver
process is launched for each entry. When ``ursim_type`` and ``driver_type`` differ,
the driver is configured for a different robot than the one running in URSim and
must therefore refuse to leave the ``unconfigured`` state.
"""

import logging
import os
import sys
import time
import unittest

import launch_testing
import pytest
import rclpy
from lifecycle_msgs.msg import State
from rclpy.node import Node

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    ControllerManagerInterface,
    DashboardInterface,
    IoStatusInterface,
    generate_driver_test_description_for_model,
)

# (ursim_type, driver_type) tuples. When the two values are equal, the driver is
# expected to come up cleanly. When they differ, the hardware interface should
# detect the mismatch in ``on_configure`` and stay in ``unconfigured``.
ROBOT_MODEL_CASES = [
    ("ur3", "ur3"),
    ("ur3e", "ur3e"),
    ("ur5", "ur5"),
    ("ur5e", "ur5e"),
    ("ur7e", "ur7e"),
    ("ur8long", "ur8long"),
    ("ur10", "ur10"),
    ("ur10e", "ur10e"),
    ("ur12e", "ur12e"),
    ("ur15", "ur15"),
    ("ur16e", "ur16e"),
    ("ur18", "ur18"),
    ("ur20", "ur20"),
    ("ur30", "ur30"),
    # Negative case: URSim runs a UR5e but the driver is configured for a UR20.
    # ``verify_robot_model`` is enabled by default in ``ur.urdf.xacro``, so the
    # hardware interface should reject the configuration.
    ("ur5e", "ur20"),
]


@pytest.mark.launch_test
@launch_testing.parametrize("ursim_type, driver_type", ROBOT_MODEL_CASES)
def generate_test_description(ursim_type, driver_type):
    return generate_driver_test_description_for_model(ur_type=driver_type, ursim_type=ursim_type)


class RobotModelStartupTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("robot_model_startup_test")
        time.sleep(1)
        # The controller_manager service is always available, regardless of whether
        # the hardware interface actually reaches the active state.
        cls._controller_manager_interface = ControllerManagerInterface(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _check_active_hardware_interface(self, ursim_type, driver_type):
        """Bring the robot up and assert the hardware component is active."""
        # The IO controller (and therefore ``resend_robot_program``) is only
        # available when the hardware interface configured successfully, so we
        # connect to it lazily here.
        dashboard_interface = DashboardInterface(self.node)
        io_status_controller_interface = IoStatusInterface(self.node)

        dashboard_interface.start_robot()
        time.sleep(1)
        self.assertTrue(
            io_status_controller_interface.resend_robot_program().success,
            f"Could not resend robot program for ursim={ursim_type}, driver={driver_type}",
        )

        hardware_info = self._controller_manager_interface.list_hardware_components()
        self.assertIsNotNone(
            hardware_info,
            f"No hardware info reported for ursim={ursim_type}, driver={driver_type}",
        )
        self.assertEqual(
            len(hardware_info.component),
            1,
            f"Expected exactly one hardware component for ursim={ursim_type}, "
            f"driver={driver_type}, got {len(hardware_info.component)}",
        )

        component = hardware_info.component[0]
        self.assertEqual(
            component.state.id,
            State.PRIMARY_STATE_ACTIVE,
            f"Hardware component '{component.name}' for ursim={ursim_type}, "
            f"driver={driver_type} is not active (state id={component.state.id})",
        )

        for interface in component.command_interfaces:
            self.assertTrue(
                interface.is_available,
                f"Command interface {interface.name} not available for "
                f"ursim={ursim_type}, driver={driver_type}",
            )
        for interface in component.state_interfaces:
            self.assertTrue(
                interface.is_available,
                f"State interface {interface.name} not available for "
                f"ursim={ursim_type}, driver={driver_type}",
            )

    def _check_unconfigured_hardware_interface(self, ursim_type, driver_type):
        """
        Assert the hardware component never rises above ``unconfigured``.

        With ``verify_robot_model`` enabled (the default), the hardware interface's
        ``on_configure`` returns ERROR when the driver's ``ur_type`` does not match
        the robot reported by URSim, leaving the component in
        ``PRIMARY_STATE_UNCONFIGURED``. Poll for a generous amount of time to make
        sure we don't accidentally race the state transition.
        """
        timeout = 60.0
        end_time = time.time() + timeout
        last_state = None
        while time.time() < end_time:
            hardware_info = self._controller_manager_interface.list_hardware_components()
            if hardware_info is not None and len(hardware_info.component) == 1:
                last_state = hardware_info.component[0].state.id
                self.assertNotEqual(
                    last_state,
                    State.PRIMARY_STATE_ACTIVE,
                    f"Hardware interface unexpectedly reached ACTIVE for ursim={ursim_type}, "
                    f"driver={driver_type} (model mismatch should have prevented this)",
                )
                self.assertNotEqual(
                    last_state,
                    State.PRIMARY_STATE_INACTIVE,
                    f"Hardware interface unexpectedly reached INACTIVE for ursim={ursim_type}, "
                    f"driver={driver_type} (model mismatch should have kept it unconfigured)",
                )
            time.sleep(2.0)

        self.assertEqual(
            last_state,
            State.PRIMARY_STATE_UNCONFIGURED,
            f"Hardware interface for ursim={ursim_type}, driver={driver_type} ended in "
            f"state id={last_state}, expected PRIMARY_STATE_UNCONFIGURED "
            f"({State.PRIMARY_STATE_UNCONFIGURED}) due to model mismatch",
        )

    def test_driver_starts_for_robot_model(self, ursim_type, driver_type):
        """
        Verify driver behavior against URSim for the parametrized model pair.

        - When ``ursim_type == driver_type``: the driver should configure and
          activate the hardware component normally and all interfaces should be
          available.
        - When they differ: the hardware interface must reject the configuration
          and remain in ``PRIMARY_STATE_UNCONFIGURED`` (verified by
          ``verify_robot_model``, which is enabled by default).
        """
        logging.info(
            "Running case ursim=%s, driver=%s (mismatch=%s)",
            ursim_type,
            driver_type,
            ursim_type != driver_type,
        )
        if ursim_type == driver_type:
            self._check_active_hardware_interface(ursim_type, driver_type)
        else:
            self._check_unconfigured_hardware_interface(ursim_type, driver_type)
