#!/usr/bin/env python
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

import os
import subprocess
import sys
import time
import unittest

import pytest

import launch_testing
import rclpy
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from sensor_msgs.msg import JointState

from lifecycle_msgs.msg import State

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    ControllerManagerInterface,
    DashboardInterface,
    IoStatusInterface,
    generate_driver_test_description,
)

URSIM_CONTAINER_NAME = "ursim_component_lifecycle_read_fault"


@pytest.mark.launch_test
@launch_testing.parametrize(
    "tf_prefix",
    [""],
)
def generate_test_description(tf_prefix):
    return generate_driver_test_description(
        tf_prefix=tf_prefix,
        ursim_container_name=URSIM_CONTAINER_NAME,
    )


class ComponentLifecycleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = Node("component_lifecycle_test")
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

    def setUp(self):
        self._dashboard_interface.start_robot()
        time.sleep(1)
        self.assertTrue(self._io_status_controller_interface.resend_robot_program().success)

    #
    # Tests
    #

    def test_component_lifecycle(self):
        hardware_info = self._controller_manager_interface.list_hardware_components()
        self.assertIsNotNone(hardware_info)
        self.assertEqual(len(hardware_info.component), 1)
        self.assertEqual(hardware_info.component[0].state.id, State.PRIMARY_STATE_ACTIVE)
        component_name = hardware_info.component[0].name

        command_interfaces = hardware_info.component[0].command_interfaces
        state_interfaces = hardware_info.component[0].state_interfaces

        # Check that all interfaces are available after startup
        for interface in command_interfaces:
            self.assertTrue(interface.is_available, f"Interface {interface.name} is not available")
        for interface in state_interfaces:
            self.assertTrue(interface.is_available, f"Interface {interface.name} is not available")

        self.assertTrue(
            self._controller_manager_interface.set_hardware_component_state(
                name=component_name, target_state=State(id=State.PRIMARY_STATE_INACTIVE)
            ).ok
        )
        time.sleep(2)

        # Check all interfaces are available after deactivation (This is current behavior, but docs say they should not be?)
        hardware_info = self._controller_manager_interface.list_hardware_components()
        self.assertIsNotNone(hardware_info)
        self.assertEqual(hardware_info.component[0].state.id, State.PRIMARY_STATE_INACTIVE)
        command_interfaces = hardware_info.component[0].command_interfaces
        state_interfaces = hardware_info.component[0].state_interfaces
        for interface in command_interfaces:
            self.assertTrue(
                interface.is_available,
                f"Interface {interface.name} is not available after deactivation",
            )
        for interface in state_interfaces:
            self.assertTrue(
                interface.is_available,
                f"Interface {interface.name} is not available after deactivation",
            )

        self.assertTrue(
            self._controller_manager_interface.set_hardware_component_state(
                name=component_name, target_state=State(id=State.PRIMARY_STATE_UNCONFIGURED)
            ).ok
        )
        time.sleep(2)

        # Check all interfaces are unavailable after shutdown of hardware
        hardware_info = self._controller_manager_interface.list_hardware_components()
        self.assertIsNotNone(hardware_info)
        self.assertEqual(hardware_info.component[0].state.id, State.PRIMARY_STATE_UNCONFIGURED)
        command_interfaces = hardware_info.component[0].command_interfaces
        state_interfaces = hardware_info.component[0].state_interfaces
        for interface in command_interfaces:
            self.assertFalse(
                interface.is_available, f"Interface {interface.name} is available after shutdown"
            )
        for interface in state_interfaces:
            self.assertFalse(
                interface.is_available, f"Interface {interface.name} is available after shutdown"
            )

        self.assertTrue(
            self._controller_manager_interface.set_hardware_component_state(
                name=component_name, target_state=State(id=State.PRIMARY_STATE_ACTIVE)
            ).ok
        )
        time.sleep(2)

        # Check all interfaces are available after reactivation
        hardware_info = self._controller_manager_interface.list_hardware_components()
        self.assertIsNotNone(hardware_info)
        self.assertEqual(hardware_info.component[0].state.id, State.PRIMARY_STATE_ACTIVE)
        command_interfaces = hardware_info.component[0].command_interfaces
        state_interfaces = hardware_info.component[0].state_interfaces
        for interface in command_interfaces:
            self.assertTrue(
                interface.is_available,
                f"Interface {interface.name} is not available after reactivation",
            )
        for interface in state_interfaces:
            self.assertTrue(
                interface.is_available,
                f"Interface {interface.name} is not available after reactivation",
            )

    def test_read_error_recovers_after_reconfigure(self):
        hardware_info = self._controller_manager_interface.list_hardware_components()
        self.assertIsNotNone(hardware_info)
        self.assertEqual(len(hardware_info.component), 1)
        component_name = hardware_info.component[0].name
        self.assertEqual(hardware_info.component[0].state.id, State.PRIMARY_STATE_ACTIVE)

        joint_states = []
        subscription = self.node.create_subscription(
            JointState, "/joint_states", joint_states.append, 10
        )
        try:
            deadline = time.monotonic() + 10.0
            while not joint_states and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            self.assertTrue(joint_states, "Timed out waiting for initial /joint_states")
            initial_stamp = joint_states[-1].header.stamp
            initial_stamp_ns = initial_stamp.sec * 1_000_000_000 + initial_stamp.nanosec

            paused = False
            try:
                subprocess.run(["docker", "pause", URSIM_CONTAINER_NAME], check=True)
                paused = True

                deadline = time.monotonic() + 120.0
                last_state = None
                while time.monotonic() < deadline:
                    hardware_info = self._controller_manager_interface.list_hardware_components()
                    if hardware_info is not None and len(hardware_info.component) == 1:
                        last_state = hardware_info.component[0].state.id
                        if last_state == State.PRIMARY_STATE_UNCONFIGURED:
                            break
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                    time.sleep(0.1)

                self.assertEqual(
                    last_state,
                    State.PRIMARY_STATE_UNCONFIGURED,
                    "Read timeout did not transition hardware to UNCONFIGURED",
                )
            finally:
                if paused:
                    subprocess.run(["docker", "unpause", URSIM_CONTAINER_NAME], check=True)

            self.assertTrue(
                self._controller_manager_interface.set_hardware_component_state(
                    name=component_name, target_state=State(id=State.PRIMARY_STATE_ACTIVE)
                ).ok
            )

            deadline = time.monotonic() + 30.0
            active_component = None
            while time.monotonic() < deadline:
                hardware_info = self._controller_manager_interface.list_hardware_components()
                if (
                    hardware_info is not None
                    and len(hardware_info.component) == 1
                    and hardware_info.component[0].state.id == State.PRIMARY_STATE_ACTIVE
                ):
                    active_component = hardware_info.component[0]
                    break
                rclpy.spin_once(self.node, timeout_sec=0.1)
                time.sleep(0.1)

            self.assertIsNotNone(active_component)
            if active_component is None:
                self.fail("Hardware did not return to ACTIVE after reconfiguration")
            self.assertEqual(
                active_component.state.id,
                State.PRIMARY_STATE_ACTIVE,
                "Hardware did not return to ACTIVE after reconfiguration",
            )
            for interface in active_component.command_interfaces:
                self.assertTrue(
                    interface.is_available,
                    f"Interface {interface.name} is not available after read-error recovery",
                )
            for interface in active_component.state_interfaces:
                self.assertTrue(
                    interface.is_available,
                    f"Interface {interface.name} is not available after read-error recovery",
                )

            self._dashboard_interface.start_robot()
            # The read error deactivated every controller using the hardware, including the broadcaster.
            self.assertTrue(
                self._controller_manager_interface.switch_controller(
                    strictness=SwitchController.Request.STRICT,
                    activate_controllers=["io_and_status_controller", "joint_state_broadcaster"],
                ).ok
            )
            self.assertTrue(self._io_status_controller_interface.resend_robot_program().success)

            joint_states.clear()
            deadline = time.monotonic() + 10.0
            while not joint_states and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            self.assertTrue(joint_states, "Timed out waiting for recovered /joint_states")
            recovered_stamp = joint_states[-1].header.stamp
            recovered_stamp_ns = recovered_stamp.sec * 1_000_000_000 + recovered_stamp.nanosec
            self.assertGreater(
                recovered_stamp_ns,
                initial_stamp_ns,
                "Joint-state timestamp did not advance after read-error recovery",
            )
        finally:
            self.node.destroy_subscription(subscription)
