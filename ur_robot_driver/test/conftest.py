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
import time

import pytest
import rclpy.node
from controller_manager_msgs.srv import SwitchController
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from ur_dashboard_msgs.msg import RobotMode
from ur_dashboard_msgs.srv import (
    GetLoadedProgram,
    GetProgramState,
    GetRobotMode,
    IsProgramRunning,
    Load,
)

TIMEOUT_WAIT_SERVICE = 10
TIMEOUT_WAIT_ACTION = 10
# If we download the docker image simultaneously to the tests, it can take quite some time until the
# dashboard server is reachable and usable.
TIMEOUT_WAIT_SERVICE_INITIAL = 120


def wait_for_service(node, srv_name, srv_type, timeout=TIMEOUT_WAIT_SERVICE):
    client = node.create_client(srv_type, srv_name)

    logging.info("Waiting for service '%s' with timeout %fs...", srv_name, timeout)

    assert client.wait_for_service(timeout)
    logging.info("  done")

    return client


def call_service(node, client, request):
    logging.info("Calling service '%s' with request: %s", client.srv_name, request)

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    assert future.result is not None
    logging.info("  Received result: %s", future.result())
    return future.result()


def wait_for_action(node, action_name, action_type, timeout=TIMEOUT_WAIT_ACTION):
    client = ActionClient(node, action_type, action_name)

    logging.info("Waiting for action server '%s' with timeout %fs...", action_name, timeout)
    assert client.wait_for_server(timeout)
    logging.info("  done")

    return client


def call_action(node, client, goal):
    logging.info("Sending goal to action server '%s': %s", client._action_name, goal)
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)

    assert future.result() is not None
    logging.info(
        "  Received response: accepted=%r, status=%d",
        future.result().accepted,
        future.result().status,
    )

    return future.result()


def get_action_result(node, client, goal_response, timeout):
    logging.info(
        "Waiting for result of action call '%s' with timeout %fs...", client._action_name, timeout
    )
    future = goal_response.get_result_async()
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)

    assert future.result() is not None
    logging.info("  Received result: %s", future.result())

    return future.result().result


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


@pytest.fixture(scope="class")
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
        return call_service(self._node, self._service_interfaces[name], request)


@pytest.fixture(scope="class")
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


class ControllerManagerInterface:
    def __init__(self, node):
        self._node = node
        self._switch_controller_client = wait_for_service(
            node, "/controller_manager/switch_controller", SwitchController
        )

    def switch_controller(self, activate_controllers=[], deactivate_controllers=[], strict=False):
        req = SwitchController.Request()
        req.activate_controllers = activate_controllers
        req.deactivate_controllers = deactivate_controllers
        req.strictness = (
            SwitchController.Request.STRICT if strict else SwitchController.Request.BEST_EFFORT
        )

        result = call_service(self._node, self._switch_controller_client, req)

        assert result.ok
        logging.info("  Successfully switched controllers")


@pytest.fixture(scope="class")
def controller_manager_interface(node):
    return ControllerManagerInterface(node)


@pytest.fixture(scope="class")
def robot_program_running(node, dashboard_interface):
    """
    Makes sure the robot program is running.

    This powers the robot on and releases its brakes. It also stops the robot program and resends
    it. This is necessary as (depending on the exact startup sequence), the program might be sent by
    headless mode at a time where it is not yet able to execute, leaving it paused.
    """
    resend_robot_program_client = wait_for_service(
        node, "/io_and_status_controller/resend_robot_program", Trigger
    )

    assert dashboard_interface.call_service("power_on", Trigger.Request()).success
    assert dashboard_interface.call_service("brake_release", Trigger.Request()).success

    time.sleep(1)

    robot_mode = dashboard_interface.call_service("get_robot_mode", GetRobotMode.Request())
    assert robot_mode.success
    assert robot_mode.robot_mode.mode == RobotMode.RUNNING

    assert dashboard_interface.call_service("stop", Trigger.Request()).success

    time.sleep(1)

    assert call_service(node, resend_robot_program_client, Trigger.Request()).success
