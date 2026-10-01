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
import signal
import sys
import time
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import launch_testing
from launch_testing.actions import ReadyToTest
import pytest


@pytest.mark.launch_test
def generate_test_description():
    dashboard_client = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        parameters=[{"robot_ip": "127.0.0.1", "autoconnect": True}],
    )
    # Keep the launch service alive after the process under test exits so post-shutdown assertions
    # can run normally.
    keepalive = ExecuteProcess(
        cmd=[sys.executable, "-c", "import time; time.sleep(30)"],
        name="keepalive",
    )

    return (
        LaunchDescription([dashboard_client, keepalive, ReadyToTest()]),
        {"dashboard_client": dashboard_client},
    )


class TestInterruptibleStartup(unittest.TestCase):

    def test_sigint_interrupts_connection(self, proc_info, proc_output, dashboard_client):
        proc_info.assertWaitForStartup(dashboard_client, timeout=5)
        proc_output.assertWaitFor(
            "Starting primary client pipeline",
            process=dashboard_client,
            timeout=5,
        )
        # Let DashboardClientROS::connect() enter its blocking primary-interface connection.
        time.sleep(0.5)

        process_event = proc_info[dashboard_client]
        start = time.monotonic()
        os.kill(process_event.pid, signal.SIGINT)
        proc_info.assertWaitForShutdown(dashboard_client, timeout=3)

        self.assertLess(time.monotonic() - start, 3)


@launch_testing.post_shutdown_test()
class TestInterruptibleStartupExitCode(unittest.TestCase):

    def test_exit_code(self, proc_info, dashboard_client):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=dashboard_client, allowable_exit_codes=[0]
        )
