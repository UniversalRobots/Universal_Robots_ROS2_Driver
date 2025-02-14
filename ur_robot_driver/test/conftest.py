#!/usr/bin/env python3
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

from test_common import (
    interfaces,
    generate_driver_test_description,
)
import pytest
import launch_pytest
import logging
import rclpy.node

# fixture scope for all fixtures
scope = "function"


# Fixtures for use with the integration tests.
@pytest.fixture(scope=scope, params=["scaled_joint_trajectory_controller"])
def initial_joint_controller(request):
    return request.param


@pytest.fixture(scope=scope, params=["", "my_ur_"])
def tf_prefix(request):
    return request.param


@pytest.fixture(scope=scope, params=["false", "true"])
def mock_hardware(request):
    return request.param


@pytest.fixture(scope=scope, params=[0, 1])
def params(request):
    params = [
        {"tf_prefix": "", "mock_hardware": "false"},
        {"tf_prefix": "my_ur_", "mock_hardware": "true"},
    ]
    return params[request.param]


# Could also be passed like this, but that result in each test being executed 4 times, instead of 2, which seems excessive:
#   @launch_pytest.fixture(scope=scope, autouse=True, auto_shutdown=True)
#   def launch_description(tf_prefix, mock_hardware):
#       return generate_driver_test_description(tf_prefix=tf_prefix, mock_hardware=mock_hardware)


# This only gives 2 exeecutions of each test
@launch_pytest.fixture(scope=scope, autouse=True, auto_shutdown=True)
def launch_description(params):
    print("-------------LAUNCHING DRIVER -------------------------------")
    tf_prefix = params["tf_prefix"]
    mock_hardware = params["mock_hardware"]
    return generate_driver_test_description(tf_prefix=tf_prefix, mock_hardware=mock_hardware)


@pytest.fixture(scope=scope)
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


@pytest.fixture(scope=scope)
def node(rclpy_init):
    """
    Creates a node with a given name.

    The name needs to be passed in as a parameter
    """
    logging.info("Creating testing node")
    rclpy_node = rclpy.node.Node("robot_driver_test")

    yield rclpy_node

    logging.info("Destroying testing node")
    rclpy_node.destroy_node()


@pytest.fixture(scope=scope)
def robot(node, params):
    interface = interfaces(node, params)
    interface.init_interfaces()
    yield interface
