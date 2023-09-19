#!/usr/bin/env python
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

import pytest
import time
import unittest

import rclpy
import rclpy.node
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from launch_testing.actions import ReadyToTest
from std_srvs.srv import Trigger
from std_msgs.msg import String as StringMsg
from ur_dashboard_msgs.msg import RobotMode
from ur_dashboard_msgs.srv import (
    GetLoadedProgram,
    GetProgramState,
    GetRobotMode,
    IsProgramRunning,
    Load,
)
from ur_msgs.msg import IOStates
from controller_manager_msgs.srv import ListControllers


ROBOT_IP = "192.168.56.101"
TIMEOUT_WAIT_SERVICE = 10
# If we download the docker image simultaneously to the tests, it can take quite some time until the
# dashboard server is reachable and usable.
TIMEOUT_WAIT_SERVICE_INITIAL = 120


@pytest.mark.launch_test
def generate_test_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20"],
        )
    )

    ur_type = LaunchConfiguration("ur_type")

    robot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "robot_ip": "192.168.56.101",
            "ur_type": ur_type,
            "launch_rviz": "false",
            "controller_spawner_timeout": str(TIMEOUT_WAIT_SERVICE_INITIAL),
            "initial_joint_controller": "scaled_joint_trajectory_controller",
            "headless_mode": "true",
            "launch_dashboard_client": "false",
            "start_joint_controller": "false",
        }.items(),
    )

    ursim = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [
                    FindPackagePrefix("ur_client_library"),
                    "lib",
                    "ur_client_library",
                    "start_ursim.sh",
                ]
            ),
            " ",
            "-m ",
            ur_type,
        ],
        name="start_ursim",
        output="screen",
    )

    wait_dashboard_server = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [FindPackagePrefix("ur_robot_driver"), "bin", "wait_dashboard_server.sh"]
            )
        ],
        name="wait_dashboard_server",
        output="screen",
    )

    driver_starter = RegisterEventHandler(
        OnProcessExit(target_action=wait_dashboard_server, on_exit=robot_driver)
    )

    return LaunchDescription(
        declared_arguments + [ReadyToTest(), wait_dashboard_server, driver_starter, ursim]
    )


class URScriptInterfaceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = rclpy.node.Node("urscript_interface_test")
        cls.init_robot(cls)

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        cls.node.destroy_node()
        rclpy.shutdown()

    def init_robot(self):
        # We wait longer for the first client, as the robot is still starting up
        power_on_client = waitForService(
            self.node, "/dashboard_client/power_on", Trigger, timeout=TIMEOUT_WAIT_SERVICE_INITIAL
        )

        # Connect to all other expected services
        dashboard_interfaces = {
            "/dashboard_client/power_off": Trigger,
            "/dashboard_client/brake_release": Trigger,
            "/dashboard_client/unlock_protective_stop": Trigger,
            "/dashboard_client/restart_safety": Trigger,
            "/dashboard_client/get_robot_mode": GetRobotMode,
            "/dashboard_client/load_installation": Load,
            "/dashboard_client/load_program": Load,
            "/dashboard_client/close_popup": Trigger,
            "/dashboard_client/get_loaded_program": GetLoadedProgram,
            "/dashboard_client/program_state": GetProgramState,
            "/dashboard_client/program_running": IsProgramRunning,
            "/dashboard_client/play": Trigger,
            "/dashboard_client/stop": Trigger,
        }
        self.service_clients = {
            srv_name: waitForService(self.node, f"{srv_name}", srv_type)
            for (srv_name, srv_type) in dashboard_interfaces.items()
        }

        self.service_clients["/controller_manager/list_controllers"] = waitForService(
            self.node,
            "/controller_manager/list_controllers",
            ListControllers,
            timeout=TIMEOUT_WAIT_SERVICE_INITIAL,
        )

        # Add first client to dict
        self.service_clients["/dashboard_client/power_on"] = power_on_client

        self.urscript_pub = self.node.create_publisher(
            StringMsg, "/urscript_interface/script_command", 1
        )

    def setUp(self):
        # Start robot
        empty_req = Trigger.Request()
        self.call_service("/dashboard_client/power_on", empty_req)
        self.call_service("/dashboard_client/brake_release", empty_req)

        time.sleep(1)
        robot_mode_resp = self.call_service(
            "/dashboard_client/get_robot_mode", GetRobotMode.Request()
        )
        self.assertEqual(robot_mode_resp.robot_mode.mode, RobotMode.RUNNING)
        self.call_service("/dashboard_client/stop", empty_req)
        time.sleep(1)

        io_controller_running = False

        while not io_controller_running:
            time.sleep(1)
            response = self.call_service(
                "/controller_manager/list_controllers", ListControllers.Request()
            )
            for controller in response.controller:
                if controller.name == "io_and_status_controller":
                    io_controller_running = controller.state == "active"

    def test_set_io(self):
        """Test setting an IO using a direct program call."""
        self.io_states_sub = self.node.create_subscription(
            IOStates,
            "/io_and_status_controller/io_states",
            self.io_msg_cb,
            rclpy.qos.qos_profile_system_default,
        )

        self.set_digout_checked(0, True)
        time.sleep(1)
        self.set_digout_checked(0, False)

        self.io_msg = None
        self.io_states_sub = self.node.create_subscription(
            IOStates,
            "/io_and_status_controller/io_states",
            self.io_msg_cb,
            rclpy.qos.qos_profile_system_default,
        )

        script_msg = StringMsg(
            data="sec my_program():\n  set_digital_out(0, False)\n  set_digital_out(1,True)\nend"
        )
        self.urscript_pub.publish(script_msg)
        self.check_pin_states([0, 1], [False, True])

        time.sleep(1)

        script_msg = StringMsg(
            data="sec my_program():\n  set_digital_out(0, True)\n  set_digital_out(1,False)\nend"
        )
        self.urscript_pub.publish(script_msg)
        self.check_pin_states([0, 1], [True, False])

    def io_msg_cb(self, msg):
        self.io_msg = msg

    def set_digout_checked(self, pin, state):
        self.io_msg = None

        script_msg = StringMsg(data=f"set_digital_out({pin}, {state})")
        self.urscript_pub.publish(script_msg)

        self.check_pin_states([pin], [state])

    def check_pin_states(self, pins, states):
        pin_states = [not x for x in states]
        end_time = time.time() + 50
        while pin_states != states and time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.io_msg is not None:
                for i, pin_id in enumerate(pins):
                    pin_states[i] = self.io_msg.digital_out_states[pin_id].state
        self.assertIsNotNone(self.io_msg, "Did not receive an IO state in requested time.")
        self.assertEqual(pin_states, states)

    def call_service(self, srv_name, request):
        self.node.get_logger().info(f"Calling service '{srv_name}' with request {request}")
        future = self.service_clients[srv_name].call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            self.node.get_logger().info(f"Received result {future.result()}")
            return future.result()
        else:
            raise Exception(f"Exception while calling service: {future.exception()}")


def waitForService(node, srv_name, srv_type, timeout=TIMEOUT_WAIT_SERVICE):
    client = node.create_client(srv_type, srv_name)
    if client.wait_for_service(timeout) is False:
        raise Exception(f"Could not reach service '{srv_name}' within timeout of {timeout}")

    node.get_logger().info(f"Successfully connected to service '{srv_name}'")
    return client
