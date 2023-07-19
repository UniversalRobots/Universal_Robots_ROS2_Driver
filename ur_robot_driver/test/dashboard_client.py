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

import pytest
from std_srvs.srv import Trigger
from ur_dashboard_msgs.msg import RobotMode
from ur_dashboard_msgs.srv import GetRobotMode

sys.path.append(os.path.dirname(__file__))
from robot_launch_descriptions import *  # noqa: E402, F403

TIMEOUT_WAIT_SERVICE = 10
# If we download the docker image simultaneously to the tests, it can take quite some time until the
# dashboard server is reachable and usable.
TIMEOUT_WAIT_SERVICE_INITIAL = 120


@pytest.mark.launch(fixture=launch_dashboard_client)  # noqa: F405
@pytest.mark.parametrize("node", [("dashboard_client_test")], indirect=True)
class TestDashboardClient:
    def test_switch_on(self, dashboard_interface):
        """Test power on a robot."""
        # Wait until the robot is booted completely
        end_time = time.time() + 10
        mode = RobotMode.DISCONNECTED
        while mode != RobotMode.POWER_OFF and time.time() < end_time:
            time.sleep(0.1)

            result = dashboard_interface.call_service("get_robot_mode", GetRobotMode.Request())
            assert result.success
            mode = result.robot_mode.mode

        # Power on robot
        assert dashboard_interface.call_service("power_on", Trigger.Request()).success

        # Wait until robot mode changes
        end_time = time.time() + 10
        mode = RobotMode.DISCONNECTED
        while mode not in (RobotMode.IDLE, RobotMode.RUNNING) and time.time() < end_time:
            time.sleep(0.1)

            result = dashboard_interface.call_service("get_robot_mode", GetRobotMode.Request())
            assert result.success
            mode = result.robot_mode.mode

        assert mode in (RobotMode.IDLE, RobotMode.RUNNING)

        # Release robot brakes
        assert dashboard_interface.call_service("brake_release", Trigger.Request()).success

        # Wait until robot mode is RUNNING
        end_time = time.time() + 10
        mode = RobotMode.DISCONNECTED
        while mode != RobotMode.RUNNING and time.time() < end_time:
            time.sleep(0.1)
            result = dashboard_interface.call_service("get_robot_mode", GetRobotMode.Request())
            assert result.success
            mode = result.robot_mode.mode

        assert mode == RobotMode.RUNNING
