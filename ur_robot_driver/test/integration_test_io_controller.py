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

import logging
import math
import os
import sys
import time
import unittest

import pytest
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from ur_msgs.msg import IOStates, ToolDataMsg, ToolDataT2Msg
from ur_msgs.srv import SetIO

from builtin_interfaces.msg import Duration as DurationMsg

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    ControllerManagerInterface,
    DashboardInterface,
    IoStatusInterface,
    generate_driver_test_description,
)


@pytest.mark.launch_test
def generate_test_description():
    return generate_driver_test_description()


class IOControllerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = Node("io_controller_test")
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
    # Test functions
    #

    def test_set_io(self):
        """Test to set an IO and check whether it has been set."""
        # Create io callback to verify result
        io_msg = None

        def io_msg_cb(msg):
            nonlocal io_msg
            io_msg = msg

        io_states_sub = self.node.create_subscription(
            IOStates,
            "/io_and_status_controller/io_states",
            io_msg_cb,
            rclpy.qos.qos_profile_system_default,
        )

        # Set pin 0 to 1.0
        test_pin = 0

        logging.info("Setting pin %d to 1.0", test_pin)
        self._io_status_controller_interface.set_io(fun=1, pin=test_pin, state=1.0)

        # Wait until the pin state has changed
        pin_state = False
        end_time = time.time() + 5
        while not pin_state and time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if io_msg is not None:
                pin_state = io_msg.digital_out_states[test_pin].state

        self.assertEqual(pin_state, 1.0)

        # Set pin 0 to 0.0
        logging.info("Setting pin %d to 0.0", test_pin)
        self._io_status_controller_interface.set_io(fun=1, pin=test_pin, state=0.0)

        # Wait until the pin state has changed back
        end_time = time.time() + 5
        while pin_state and time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if io_msg is not None:
                pin_state = io_msg.digital_out_states[test_pin].state

        self.assertEqual(pin_state, 0.0)

        # Clean up io subscription
        self.node.destroy_subscription(io_states_sub)

    def test_set_payload(self):
        """
        Test that setting a payload succeeds and the value is verified against RTDE feedback.

        With ``verify_payload_on_set`` enabled (default for the real driver), the
        controller only reports success after the requested payload mass and
        center-of-gravity have been confirmed via the RTDE state interfaces.
        """
        # Set a non-default payload so we can detect the change reliably
        mass = 1.5
        cog = Vector3(x=0.01, y=0.02, z=0.03)

        logging.info("Setting payload to mass=%f, cog=(%f, %f, %f)", mass, cog.x, cog.y, cog.z)
        result = self._io_status_controller_interface.set_payload(mass=mass, center_of_gravity=cog)
        self.assertTrue(
            result.success,
            "set_payload returned success=False. With verify_payload_on_set=true the "
            "controller only returns success once the RTDE feedback matches the request.",
        )

        # Reset the payload to zero and verify again. This makes sure the verification
        # logic also detects subsequent changes and is not just matching the initial state.
        logging.info("Resetting payload to zero")
        result = self._io_status_controller_interface.set_payload(
            mass=0.0, center_of_gravity=Vector3(x=0.0, y=0.0, z=0.0)
        )
        self.assertTrue(result.success, "Resetting payload via set_payload failed")

    def test_set_payload_with_inertia(self):
        """Setting mass, COG and full inertia matrix should succeed."""
        res = self._io_status_controller_interface.set_payload(
            mass=1.0,
            center_of_gravity=Vector3(x=0.1, y=0.0, z=0.2),
            ixx=0.01,
            iyy=0.01,
            izz=0.02,
            ixy=0.0,
            ixz=0.0,
            iyz=0.0,
            transition_time=DurationMsg(),
        )
        self.assertTrue(res.success)

    def test_set_payload_with_transition_time(self):
        """
        Setting payload with transition_time > 0 should succeed.

        The service should wait for the transition to complete before verifying.
        """
        res = self._io_status_controller_interface.set_payload(
            mass=1.0,
            center_of_gravity=Vector3(x=0.0, y=0.0, z=0.1),
            ixx=0.01,
            iyy=0.01,
            izz=0.02,
            transition_time=DurationMsg(sec=1, nanosec=0),
        )
        self.assertTrue(res.success)

    def test_set_payload_updates_sequentially(self):
        """Multiple sequential set_payload calls should all succeed."""
        payloads = [
            {"mass": 0.5, "cx": 0.0, "cy": 0.0, "cz": 0.1, "ixx": 0.005, "iyy": 0.005, "izz": 0.01},
            {"mass": 1.0, "cx": 0.1, "cy": 0.0, "cz": 0.2, "ixx": 0.01, "iyy": 0.01, "izz": 0.02},
            {
                "mass": 2.0,
                "cx": 0.05,
                "cy": 0.05,
                "cz": 0.15,
                "ixx": 0.02,
                "iyy": 0.02,
                "izz": 0.04,
            },
            {"mass": 0.0, "cx": 0.0, "cy": 0.0, "cz": 0.0, "ixx": 0.0, "iyy": 0.0, "izz": 0.0},
        ]
        for p in payloads:
            res = self._io_status_controller_interface.set_payload(
                mass=p["mass"],
                center_of_gravity=Vector3(x=p["cx"], y=p["cy"], z=p["cz"]),
                ixx=p["ixx"],
                iyy=p["iyy"],
                izz=p["izz"],
                transition_time=DurationMsg(),
            )
            self.assertTrue(res.success, f"set_payload failed for m={p['mass']}")

    def _wait_for_tool_data(self, timeout=5.0):
        """Return the next ~/tool_data message."""
        tool_msg = None

        def tool_cb(msg):
            nonlocal tool_msg
            tool_msg = msg

        subscription = self.node.create_subscription(
            ToolDataMsg,
            "/io_and_status_controller/tool_data",
            tool_cb,
            rclpy.qos.qos_profile_system_default,
        )
        end_time = time.time() + timeout
        while tool_msg is None and time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(subscription)
        self.assertIsNotNone(tool_msg, "No tool_data received")
        return tool_msg

    def _open_tool_data_t2(self):
        """Subscribe to ~/tool_data_t2 before any wait, so discovery is not the timeout."""
        state = {"msg": None}

        def tool_cb(msg):
            state["msg"] = msg

        subscription = self.node.create_subscription(
            ToolDataT2Msg,
            "/io_and_status_controller/tool_data_t2",
            tool_cb,
            rclpy.qos.qos_profile_system_default,
        )
        return subscription, state

    def _wait_for_tool_data_t2(self, state, timeout=1.0):
        """Return the latest ~/tool_data_t2 sample, or None if connector T2 is absent.

        The subscription must already exist. ~/tool_data is received first so the wait
        starts only after this node is receiving controller traffic.
        """
        self._wait_for_tool_data()
        end_time = time.time() + timeout
        while state["msg"] is None and time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return state["msg"]

    def _wait_for_t2_voltage(self, state, expected, tolerance=5.0, timeout=5.0):
        """Wait until the measured T2 voltage is within tolerance of expected.

        tool_output_voltage is a measured value: a 24 V setpoint can read 23 V, and
        switching off takes about a second to decay. ``state`` is the dict filled by
        the subscription opened for this test.
        """
        end_time = time.time() + timeout
        last = None
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            sample = state["msg"]
            if sample is None:
                continue
            last = sample.tool_output_voltage
            if not math.isnan(last) and abs(last - expected) <= tolerance:
                return last
            state["msg"] = None
        self.fail(f"T2 voltage {last} did not reach {expected} +/- {tolerance} within {timeout}s")

    def test_tool_voltage_t0_rejects_48v(self):
        """Connector T0 accepts 0, 12 and 24 V. 48 V belongs to connector T2."""
        result = self._io_status_controller_interface.set_io(
            fun=SetIO.Request.FUN_SET_TOOL_VOLTAGE,
            pin=0,
            state=float(SetIO.Request.STATE_TOOL_VOLTAGE_48V),
        )
        self.assertFalse(result.success)

    def test_tool_voltage_t2(self):
        """T2 accepts the command only when the robot reports connector T2.

        URSim 10.15 reports 0 V for every T2 setpoint, so a measured-voltage check is
        opt-in via UR_TOOL_T2_MEASURE_VOLTAGE=1 and is meant for a real robot.
        """
        subscription, t2_state = self._open_tool_data_t2()
        try:
            available = self._wait_for_tool_data_t2(t2_state) is not None
            measure = os.environ.get("UR_TOOL_T2_MEASURE_VOLTAGE") == "1"
            try:
                result = self._io_status_controller_interface.set_io(
                    fun=SetIO.Request.FUN_SET_TOOL_VOLTAGE_T2,
                    pin=0,
                    state=float(SetIO.Request.STATE_TOOL_VOLTAGE_24V),
                )
                if available:
                    self.assertTrue(result.success)
                    if measure:
                        self._wait_for_t2_voltage(t2_state, SetIO.Request.STATE_TOOL_VOLTAGE_24V)
                else:
                    self.assertFalse(result.success)
            finally:
                # Best-effort. An assertion failure, or discovery timing out, must still
                # request 0 V so a real connector is not left at 24 V. The request is only
                # required to succeed when the robot reported connector T2.
                off = self._io_status_controller_interface.set_io(
                    fun=SetIO.Request.FUN_SET_TOOL_VOLTAGE_T2,
                    pin=0,
                    state=float(SetIO.Request.STATE_TOOL_VOLTAGE_0V),
                )
            if available:
                self.assertTrue(off.success)
                if measure:
                    self._wait_for_t2_voltage(t2_state, SetIO.Request.STATE_TOOL_VOLTAGE_0V)
        finally:
            self.node.destroy_subscription(subscription)
