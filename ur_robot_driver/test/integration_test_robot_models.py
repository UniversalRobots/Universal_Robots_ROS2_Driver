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
from lifecycle_msgs.msg import State as LifecycleState
from rclpy.node import Node

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    ControllerManagerInterface,
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

HW_NAME = "ur"


@pytest.mark.launch_test
@launch_testing.parametrize("ursim_type, driver_type", ROBOT_MODEL_CASES)
def generate_test_description(ursim_type, driver_type):
    return generate_driver_test_description_for_model(
        ur_type=driver_type, ursim_type=ursim_type, hw_name=HW_NAME
    )


class RobotModelStartupTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("robot_model_startup_test")
        time.sleep(1)
        cls._controller_manager_interface = ControllerManagerInterface(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _get_hardware_component_state(self, ursim_type, driver_type):
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
        self.assertEqual(component.name, HW_NAME)
        return component.state

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

        self.assertEqual(
            self._get_hardware_component_state(ursim_type, driver_type).id,
            LifecycleState.PRIMARY_STATE_UNCONFIGURED,
        )

        target_state = LifecycleState(id=LifecycleState.PRIMARY_STATE_INACTIVE, label="inactive")
        result = self._controller_manager_interface.set_hardware_component_state(
            name=HW_NAME, target_state=target_state
        )
        if ursim_type == driver_type:
            self.assertTrue(
                result.ok,
                f"Hardware component '{HW_NAME}' for ursim={ursim_type}, "
                f"driver={driver_type} did not reach state '{target_state.label}' but is in state '{result.state.label}'",
            )
            self.assertEqual(result.state, target_state)
        else:
            self.assertFalse(
                result.ok,
                f"Hardware interface unexpectedly reached {target_state.label} for ursim={ursim_type}, "
                f"driver={driver_type} (model mismatch should have prevented this)",
            )
