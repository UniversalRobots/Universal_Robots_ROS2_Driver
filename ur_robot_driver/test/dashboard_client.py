#!/usr/bin/env python
# Copyright 2019, FZI Forschungszentrum Informatik
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
from rclpy.node import Node
from ur_dashboard_msgs.msg import RobotMode, UserRole, OperationalMode, SafetyStatus

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    DashboardInterface,
    generate_dashboard_test_description,
)


@pytest.mark.launch_test
@launch_testing.parametrize(
    "ursim_version, ur_type",
    [("latest", "ur30"), ("10.12.0", "ur15"), ("3.15.8", "ur10")],
)
def generate_test_description(ursim_version, ur_type):
    return generate_dashboard_test_description(ursim_version, ur_type)


class DashboardClientTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls, ursim_version):
        # Initialize the ROS context
        rclpy.init()
        cls.node = Node("dashboard_client_test")
        cls.init_robot(cls, ursim_version)

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        cls.node.destroy_node()
        rclpy.shutdown()

    def init_robot(self, ursim_version):
        self._dashboard_interface = DashboardInterface(self.node)
        result = self._dashboard_interface.is_in_remote_control()
        self.remote_control = result.remote_control
        if self.remote_control or ursim_version.startswith("5.") or ursim_version == "latest":
            self._dashboard_interface.power_off()  # create a defined starting state

    def test_switch_on(self, ursim_version):
        """Test power on a robot."""
        if ursim_version.startswith("10."):
            self.skipTest("Currently, this test isn't supported on PolyScope X")

        # Wait until the robot is booted completely
        end_time = time.time() + 10
        mode = RobotMode.DISCONNECTED
        while mode != RobotMode.POWER_OFF and time.time() < end_time:
            time.sleep(0.1)
            result = self._dashboard_interface.get_robot_mode()
            self.assertTrue(result.success)
            mode = result.robot_mode.mode

        # Power on robot
        self.assertTrue(self._dashboard_interface.power_on().success)

        # Wait until robot mode changes
        end_time = time.time() + 10
        mode = RobotMode.DISCONNECTED
        while mode not in (RobotMode.IDLE, RobotMode.RUNNING) and time.time() < end_time:
            time.sleep(0.1)
            result = self._dashboard_interface.get_robot_mode()
            self.assertTrue(result.success)
            mode = result.robot_mode.mode

        self.assertIn(mode, (RobotMode.IDLE, RobotMode.RUNNING))

        # Release robot brakes
        self.assertTrue(self._dashboard_interface.brake_release().success)

        # Wait until robot mode is RUNNING
        end_time = time.time() + 10
        mode = RobotMode.DISCONNECTED
        while mode != RobotMode.RUNNING and time.time() < end_time:
            time.sleep(0.1)
            result = self._dashboard_interface.get_robot_mode()
            self.assertTrue(result.success)
            mode = result.robot_mode.mode

        self.assertEqual(mode, RobotMode.RUNNING)

    def test_get_robot_mode(self):
        """Test get robot mode."""
        result = self._dashboard_interface.get_robot_mode()
        self.assertTrue(result.success)

    def test_program_management(self, ursim_version):
        """Test uploading a program."""
        if not ursim_version.startswith("10."):
            self.skipTest("Uploading a program is only supported on PolyScope X")
        result = self._dashboard_interface.upload_program(
            file_path=os.path.join(os.path.dirname(__file__), "test_program.urpx")
        )
        self.assertTrue(result.success)
        self.assertEqual(result.program_name, "test upload")

        result = self._dashboard_interface.get_programs()
        self.assertTrue(result.success)
        self.assertTrue(len(result.programs) > 0)

        # TODO: Updating a program requires an open UI session. We would need to start a browser
        # from within this test. Maybe it would be better to turn those tests into unittests, as
        # functionality is tested in the client library, already.
        # result = self._dashboard_interface.update_program(
        # file_path=os.path.join(os.path.dirname(__file__), "test_program.urpx")
        # )
        # self.assertTrue(result.success)
        # self.assertEqual(result.program_name, "test upload")

        result = self._dashboard_interface.download_program(
            program_name="test upload", target_path="/tmp/test_program_download.urpx"
        )
        self.assertTrue(result.success)

    def test_get_polyscope_version(self, ursim_version):
        if ursim_version.startswith("10."):
            self.skipTest("Getting polyscope version is not supported on PolyScope X")
        resp = self._dashboard_interface.get_polyscope_version()
        self.assertTrue(resp.success)
        self.assertNotEqual(resp.version.major, 0)

    def test_get_serial_number(self, ursim_version):
        if ursim_version.startswith("10."):
            self.skipTest("Getting serial number is not supported on PolyScope X")
        resp = self._dashboard_interface.get_serial_number()
        self.assertTrue(resp.success)
        self.assertNotEqual(resp.serial_number, 0)

    def test_user_role_services(self, ursim_version):
        if not ursim_version.startswith("3."):
            self.skipTest("User role services only supported on CB3")
        roles = [
            UserRole.PROGRAMMER,
            UserRole.OPERATOR,
            UserRole.NONE,
            UserRole.LOCKED,
            UserRole.RESTRICTED,
        ]
        for role in roles:
            req = UserRole(role=role)
            resp = self._dashboard_interface.set_user_role(user_role=req)
            self.assertTrue(resp.success)
            resp = self._dashboard_interface.get_user_role()
            self.assertTrue(resp.success)
            self.assertEqual(role, resp.user_role.role)

    # Not all operational mode services are supported in PolyScope X yet
    def test_operational_mode_services(self, ursim_version):
        if not ursim_version.startswith("5."):
            self.skipTest(
                "Operational mode services only supported on PolyScope 5 robots, skipping tests"
            )
        modes = [OperationalMode.MANUAL, OperationalMode.AUTOMATIC]
        for mode in modes:
            req = OperationalMode(mode=mode)
            resp = self._dashboard_interface.set_operational_mode(operational_mode=req)
            self.assertTrue(resp.success)
            resp = self._dashboard_interface.get_operational_mode()
            self.assertTrue(resp.success)
            self.assertEqual(mode, resp.operational_mode.mode)
        resp = self._dashboard_interface.clear_operational_mode()
        self.assertTrue(resp.success)

    def test_get_operational_mode(self, ursim_version):
        if not ursim_version.startswith("10."):
            self.skipTest("Specific test for PolyScope X, skipping")
        resp = self._dashboard_interface.get_operational_mode()
        self.assertTrue(resp.success)
        self.assertIn(resp.operational_mode.mode, ["MANUAL", "AUTOMATIC"])

    def test_get_robot_model(self, ursim_version):
        if ursim_version.startswith("10."):
            self.skipTest("Getting robot model not supported on PolyScope X, skipping tests")
        resp = self._dashboard_interface.get_robot_model()
        self.assertTrue(resp.success)
        self.assertTrue("UR" in resp.robot_model)

    def test_get_safety_status(self, ursim_version):
        if not ursim_version.startswith("5."):
            self.skipTest("Safety status only supported on PolyScope 5 robots, skipping tests")
        resp = self._dashboard_interface.get_safety_status()
        self.assertTrue(resp.success)
        self.assertEqual(resp.safety_status.status, SafetyStatus.NORMAL)

    def test_generate_flight_report(self, ursim_version):
        if ursim_version.startswith("10."):
            self.skipTest("Generating flight report not supported on PolyScope X, skipping tests")
        resp = self._dashboard_interface.generate_flight_report()
        self.assertTrue(resp.success)
        self.assertNotEqual(resp.report_id, "")

    def test_generate_support_file(self, ursim_version):
        if ursim_version.startswith("10."):
            self.skipTest("Generating support file not supported on PolyScope X, skipping tests")
        resp = self._dashboard_interface.generate_support_file()
        self.assertTrue(resp.success)
        self.assertNotEqual(resp.generated_file_name, "")
