#!/usr/bin/env python
# Copyright 2026, Shin
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
import signal
import socket
import unittest

import launch
import launch.events.process
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_ros.actions
import pytest


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    refusing_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    refusing_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    refusing_socket.bind(("127.0.0.1", 30001))

    urscript_interface = launch_ros.actions.Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": "127.0.0.1"}],
        output="screen",
    )

    return (
        launch.LaunchDescription(
            [
                urscript_interface,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "refusing_socket": refusing_socket,
            "urscript_interface": urscript_interface,
        },
    )


class TestStartupShutdown(unittest.TestCase):
    def test_sigint_interrupts_primary_client_startup(
        self, launch_service, proc_info, proc_output, refusing_socket, urscript_interface
    ):
        try:
            proc_info.assertWaitForStartup(process=urscript_interface, timeout=5)
            proc_output.assertWaitFor(
                "Retrying in 10 seconds",
                process=urscript_interface,
                stream="stdout",
                timeout=10,
            )

            launch_service.emit_event(
                launch.events.process.SignalProcess(
                    signal_number=signal.SIGINT,
                    process_matcher=lambda process: process is urscript_interface,
                )
            )

            proc_info.assertWaitForShutdown(process=urscript_interface, timeout=5)
            launch_testing.asserts.assertExitCodes(
                proc_info,
                process=urscript_interface,
                allowable_exit_codes=[0],
            )
        finally:
            refusing_socket.close()
