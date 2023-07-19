# Copyright 2023, FZI Forschungszentrum Informatik
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

import pytest
import rclpy.node
from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import (
    GetLoadedProgram,
    GetProgramState,
    GetRobotMode,
    IsProgramRunning,
    Load,
)

TIMEOUT_WAIT_SERVICE = 10
# If we download the docker image simultaneously to the tests, it can take quite some time until the
# dashboard server is reachable and usable.
TIMEOUT_WAIT_SERVICE_INITIAL = 120


def wait_for_service(node, srv_name, srv_type, timeout=TIMEOUT_WAIT_SERVICE):
    client = node.create_client(srv_type, srv_name)

    logging.info("Waiting for service '%s' with timeout %fs...", srv_name, timeout)
    if client.wait_for_service(timeout) is False:
        raise Exception(f"Could not reach service '{srv_name}' within timeout of {timeout}")
    logging.info("  done")

    return client


@pytest.fixture(scope="module")
def rclpy_init():
    """
    Initializes and finalizes rclpy.

    This ensures that rclpy.init() has been called before, but is not called more than once.
    """
    logging.info("Initializing rclpy")
    rclpy.init()

    yield

    logging.info("Shutting down rclpy")
    rclpy.shutdown()


@pytest.fixture()
def node(request, rclpy_init):
    """
    Creates a node with a given name.

    The name needs to be passed in as a parameter
    """
    logging.info("Creating node with name '%s'", request.param)
    rclpy_node = rclpy.node.Node(request.param)

    yield rclpy_node

    logging.info("Destroying node '%s'", request.param)
    rclpy_node.destroy_node()


class DashboardClient:
    def __init__(self, node, service_interfaces):
        self._node = node
        self._service_interfaces = service_interfaces

    def call_service(self, name, request):
        logging.info("Calling dashboard service '%s' with request: %s", name, request)

        future = self._service_interfaces[name].call_async(request)
        rclpy.spin_until_future_complete(self._node, future)

        if future.result() is None:
            raise Exception(f"Exception while calling service: {future.exception()}")

        logging.info("  Received result: %s", future.result())
        return future.result()


@pytest.fixture()
def dashboard_interface(node):
    # We wait longer for the first client, as the robot is still starting up
    power_on_client = wait_for_service(
        node,
        "/dashboard_client/power_on",
        Trigger,
        timeout=TIMEOUT_WAIT_SERVICE_INITIAL,
    )

    # Connect to all other expected services
    dashboard_interfaces = {
        "power_off": Trigger,
        "brake_release": Trigger,
        "unlock_protective_stop": Trigger,
        "restart_safety": Trigger,
        "get_robot_mode": GetRobotMode,
        "load_installation": Load,
        "load_program": Load,
        "close_popup": Trigger,
        "get_loaded_program": GetLoadedProgram,
        "program_state": GetProgramState,
        "program_running": IsProgramRunning,
        "play": Trigger,
        "stop": Trigger,
    }

    dashboard_clients = {
        srv_name: wait_for_service(node, f"/dashboard_client/{srv_name}", srv_type)
        for (srv_name, srv_type) in dashboard_interfaces.items()
    }

    # Add first client to dict
    dashboard_clients["power_on"] = power_on_client

    return DashboardClient(node, dashboard_clients)
