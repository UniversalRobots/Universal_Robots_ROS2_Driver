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

# ----------------------------------------------------------------------
# \file
#
# \author  Rune Søe-Knudsen rsk@universal-robots.com
# \date    2026-03-02
#
# ----------------------------------------------------------------------

import os
import sys
import time
import unittest

import pytest

import launch_testing
import rclpy
from rclpy.node import Node

from controller_manager_msgs.srv import SwitchController
from ur_msgs.msg import FrictionModelParameters

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    ControllerManagerInterface,
    DashboardInterface,
    FrictionModelInterface,
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


class FrictionModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = Node("friction_model_test")
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

        self._controller_manager_interface.wait_for_controller("friction_model_controller")

    def setUp(self):
        self._dashboard_interface.start_robot()
        time.sleep(1)
        self.assertTrue(self._io_status_controller_interface.resend_robot_program().success)
        time.sleep(1)

    def test_set_friction_model_parameters(self, tf_prefix):
        """Test setting friction model parameters with valid values."""
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=[
                    "friction_model_controller",
                ],
                deactivate_controllers=[
                    "scaled_joint_trajectory_controller",
                    "joint_trajectory_controller",
                ],
            ).ok
        )

        self._friction_model_interface = FrictionModelInterface(self.node)

        res = self._friction_model_interface.set_friction_model_parameters(
            parameters=FrictionModelParameters(
                viscous_scale=[0.9, 0.9, 0.8, 0.9, 0.9, 0.9],
                coulomb_scale=[0.8, 0.8, 0.7, 0.8, 0.8, 0.8],
            ),
        )
        self.assertTrue(res.success)

        # Deactivate controller
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                deactivate_controllers=["friction_model_controller"],
            ).ok
        )

    def test_invalid_viscous_array_length(self, tf_prefix):
        """Test that wrong viscous_scale array length is rejected."""
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=[
                    "friction_model_controller",
                ],
                deactivate_controllers=[
                    "scaled_joint_trajectory_controller",
                    "joint_trajectory_controller",
                ],
            ).ok
        )

        self._friction_model_interface = FrictionModelInterface(self.node)

        # Too few elements
        res = self._friction_model_interface.set_friction_model_parameters(
            parameters=FrictionModelParameters(
                viscous_scale=[0.9, 0.9, 0.8, 0.9, 0.9],
                coulomb_scale=[0.8, 0.8, 0.7, 0.8, 0.8, 0.8],
            ),
        )
        self.assertFalse(res.success)

        # Too many elements
        res = self._friction_model_interface.set_friction_model_parameters(
            parameters=FrictionModelParameters(
                viscous_scale=[0.9, 0.9, 0.8, 0.9, 0.9, 0.9, 0.9],
                coulomb_scale=[0.8, 0.8, 0.7, 0.8, 0.8, 0.8],
            ),
        )
        self.assertFalse(res.success)

        # Deactivate controller
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                deactivate_controllers=["friction_model_controller"],
            ).ok
        )

    def test_invalid_coulomb_array_length(self, tf_prefix):
        """Test that wrong coulomb_scale array length is rejected."""
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=[
                    "friction_model_controller",
                ],
                deactivate_controllers=[
                    "scaled_joint_trajectory_controller",
                    "joint_trajectory_controller",
                ],
            ).ok
        )

        self._friction_model_interface = FrictionModelInterface(self.node)

        res = self._friction_model_interface.set_friction_model_parameters(
            parameters=FrictionModelParameters(
                viscous_scale=[0.9, 0.9, 0.8, 0.9, 0.9, 0.9],
                coulomb_scale=[0.8, 0.8, 0.7],
            ),
        )
        self.assertFalse(res.success)

        # Deactivate controller
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                deactivate_controllers=["friction_model_controller"],
            ).ok
        )

    def test_set_friction_model_parameters_on_inactive_controller_fails(self, tf_prefix):
        """Test that calling the service on an inactive controller fails."""
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=[],
                deactivate_controllers=[
                    "friction_model_controller",
                    "scaled_joint_trajectory_controller",
                    "joint_trajectory_controller",
                ],
            ).ok
        )

        self._friction_model_interface = FrictionModelInterface(self.node)

        res = self._friction_model_interface.set_friction_model_parameters(
            parameters=FrictionModelParameters(
                viscous_scale=[0.9, 0.9, 0.8, 0.9, 0.9, 0.9],
                coulomb_scale=[0.8, 0.8, 0.7, 0.8, 0.8, 0.8],
            ),
        )
        self.assertFalse(res.success)
