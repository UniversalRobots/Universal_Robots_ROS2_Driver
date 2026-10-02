#!/usr/bin/env python3
# Copyright 2026, Shin
# SPDX-License-Identifier: BSD-3-Clause

"""A dashboard peer that accepts TCP but never sends its welcome message."""

import os
import signal
import socket
import struct
import sys
import threading
import time
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import launch_testing
from launch_testing.actions import ReadyToTest
import pytest


class SilentDashboardFixture:
    def __init__(self):
        self.closed = threading.Event()
        self.primary_accepted = threading.Event()
        self.dashboard_accepted = threading.Event()
        self.listeners = []
        self.threads = []
        for port, accepted, send_version in (
            (30001, self.primary_accepted, True),
            (29999, self.dashboard_accepted, False),
        ):
            listener = socket.socket()
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(("127.0.0.1", port))
            listener.listen(1)
            listener.settimeout(0.1)
            self.listeners.append(listener)
            thread = threading.Thread(
                target=self._serve, args=(listener, accepted, send_version), daemon=True
            )
            thread.start()
            self.threads.append(thread)

    def _serve(self, listener, accepted, send_version):
        connection = None
        try:
            while not self.closed.is_set():
                try:
                    connection, _ = listener.accept()
                    break
                except socket.timeout:
                    continue
            if connection is None:
                return
            accepted.set()
            if send_version:
                project = b"URControl"
                payload = struct.pack(">B", len(project)) + project
                payload += struct.pack(">BBii", 5, 12, 0, 1) + b"validation"
                body = struct.pack(">QBB", 0, 0, 3) + payload
                connection.sendall(struct.pack(">iB", 5 + len(body), 20) + body)
            connection.settimeout(0.1)
            while not self.closed.is_set():
                try:
                    if not connection.recv(4096):
                        break
                except socket.timeout:
                    continue
        finally:
            if connection is not None:
                connection.close()

    def close(self):
        self.closed.set()
        for listener in self.listeners:
            listener.close()
        for thread in self.threads:
            thread.join(timeout=1)


@pytest.mark.launch_test
def generate_test_description():
    fixture = SilentDashboardFixture()
    dashboard_client = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        parameters=[{"robot_ip": "127.0.0.1", "autoconnect": True}],
    )
    keepalive = ExecuteProcess(
        cmd=[sys.executable, "-c", "import time; time.sleep(8)"], name="keepalive"
    )
    return (
        LaunchDescription([dashboard_client, keepalive, ReadyToTest()]),
        {"dashboard_client": dashboard_client, "fixture": fixture},
    )


class TestSilentGreetingShutdown(unittest.TestCase):
    def test_sigint_exits_promptly(self, proc_info, dashboard_client, fixture):
        try:
            proc_info.assertWaitForStartup(dashboard_client, timeout=5)
            self.assertTrue(fixture.primary_accepted.wait(5), "Primary version fixture not reached")
            self.assertTrue(fixture.dashboard_accepted.wait(10), "Dashboard welcome fixture not reached")
            # Ensure the client has entered the silent welcome receive before SIGINT.
            time.sleep(0.1)
            process_event = proc_info[dashboard_client]
            started = time.monotonic()
            os.kill(process_event.pid, signal.SIGINT)
            proc_info.assertWaitForShutdown(dashboard_client, timeout=3)
            self.assertLess(time.monotonic() - started, 3)
        finally:
            fixture.close()


@launch_testing.post_shutdown_test()
class TestSilentGreetingExitCode(unittest.TestCase):
    def test_exit_code(self, proc_info, dashboard_client):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=dashboard_client, allowable_exit_codes=[0]
        )
