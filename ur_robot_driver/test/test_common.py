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

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
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
from ur_msgs.srv import SetIO

TIMEOUT_WAIT_SERVICE = 10
TIMEOUT_WAIT_SERVICE_INITIAL = 120  # If we download the docker image simultaneously to the tests, it can take quite some time until the dashboard server is reachable and usable.
TIMEOUT_WAIT_ACTION = 10


def _wait_for_service(node, srv_name, srv_type, timeout):
    client = node.create_client(srv_type, srv_name)

    logging.info("Waiting for service '%s' with timeout %fs...", srv_name, timeout)
    if client.wait_for_service(timeout) is False:
        raise Exception(f"Could not reach service '{srv_name}' within timeout of {timeout}")
    logging.info("  Successfully connected to service '%s'", srv_name)

    return client


def _wait_for_action(node, action_name, action_type, timeout):
    client = ActionClient(node, action_type, action_name)

    logging.info("Waiting for action server '%s' with timeout %fs...", action_name, timeout)
    if client.wait_for_server(timeout) is False:
        raise Exception(
            f"Could not reach action server '{action_name}' within timeout of {timeout}"
        )

    logging.info("  Successfully connected to action server '%s'", action_name)
    return client


def _call_service(node, client, request):
    logging.info("Calling service client '%s' with request '%s'", client.srv_name, request)
    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        logging.info("  Received result: %s", future.result())
        return future.result()

    raise Exception(f"Error while calling service '{client.srv_name}': {future.exception()}")


class _ServiceInterface:
    def __init__(
        self, node, initial_timeout=TIMEOUT_WAIT_SERVICE_INITIAL, timeout=TIMEOUT_WAIT_SERVICE
    ):
        self.__node = node

        self.__service_clients = {
            srv_name: (
                _wait_for_service(self.__node, srv_name, srv_type, initial_timeout),
                srv_type,
            )
            for srv_name, srv_type in self.__initial_services.items()
        }
        self.__service_clients.update(
            {
                srv_name: (_wait_for_service(self.__node, srv_name, srv_type, timeout), srv_type)
                for srv_name, srv_type in self.__services.items()
            }
        )

    def __init_subclass__(mcs, namespace="", initial_services={}, services={}, **kwargs):
        super().__init_subclass__(**kwargs)

        mcs.__initial_services = {
            namespace + "/" + srv_name: srv_type for srv_name, srv_type in initial_services.items()
        }
        mcs.__services = {
            namespace + "/" + srv_name: srv_type for srv_name, srv_type in services.items()
        }

        for srv_name, srv_type in list(initial_services.items()) + list(services.items()):
            full_srv_name = namespace + "/" + srv_name

            setattr(
                mcs,
                srv_name,
                lambda s, full_srv_name=full_srv_name, *args, **kwargs: _call_service(
                    s.__node,
                    s.__service_clients[full_srv_name][0],
                    s.__service_clients[full_srv_name][1].Request(*args, **kwargs),
                ),
            )


class ActionInterface:
    def __init__(self, node, action_name, action_type, timeout=TIMEOUT_WAIT_ACTION):
        self.__node = node

        self.__action_name = action_name
        self.__action_type = action_type
        self.__action_client = _wait_for_action(node, action_name, action_type, timeout)

    def send_goal(self, *args, **kwargs):
        goal = self.__action_type.Goal(*args, **kwargs)

        logging.info("Sending goal to action server '%s': %s", self.__action_name, goal)
        future = self.__action_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self.__node, future)

        if future.result() is not None:
            logging.info("  Received result: %s", future.result())
            return future.result()
        pass

    def get_result(self, goal_handle, timeout):
        future_res = goal_handle.get_result_async()

        logging.info(
            "Waiting for action result from '%s' with timeout %fs", self.__action_name, timeout
        )
        rclpy.spin_until_future_complete(self.__node, future_res, timeout_sec=timeout)

        if future_res.result() is not None:
            logging.info("  Received result: %s", future_res.result().result)
            return future_res.result().result
        else:
            raise Exception(
                f"Exception while calling action '{self.__action_name}': {future_res.exception()}"
            )


class DashboardInterface(
    _ServiceInterface,
    namespace="/dashboard_client",
    initial_services={
        "power_on": Trigger,
    },
    services={
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
    },
):
    def start_robot(self):
        self._check_call(self.power_on())
        self._check_call(self.brake_release())

        time.sleep(1)

        robot_mode = self.get_robot_mode()
        self._check_call(robot_mode)
        if robot_mode.robot_mode.mode != RobotMode.RUNNING:
            raise Exception(
                f"Incorrect robot mode: Expected {RobotMode.RUNNING}, got {robot_mode.robot_mode.mode}"
            )

        self._check_call(self.stop())

    def _check_call(self, result):
        if not result.success:
            raise Exception("Service call not successful")


class ControllerManagerInterface(
    _ServiceInterface,
    namespace="/controller_manager",
    initial_services={"switch_controller": SwitchController},
    services={"list_controllers": ListControllers},
):
    def wait_for_controller(self, controller_name, target_state="active"):
        while True:
            controllers = self.list_controllers().controller
            for controller in controllers:
                if (controller.name == controller_name) and (controller.state == target_state):
                    return

            time.sleep(1)


class IoStatusInterface(
    _ServiceInterface,
    namespace="/io_and_status_controller",
    initial_services={"set_io": SetIO},
    services={"resend_robot_program": Trigger},
):
    pass


def _declare_launch_arguments():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
        )
    )

    return declared_arguments


def _ursim_action():
    ur_type = LaunchConfiguration("ur_type")

    return ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [
                    FindPackagePrefix("ur_client_library"),
                    "lib",
                    "ur_client_library",
                    "start_ursim.sh",
                ]
            ),
            "-m",
            ur_type,
        ],
        name="start_ursim",
        output="screen",
    )


def generate_dashboard_test_description():
    dashboard_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_robot_driver"),
                    "launch",
                    "ur_dashboard_client.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_ip": "192.168.56.101",
        }.items(),
    )

    return LaunchDescription(
        _declare_launch_arguments() + [ReadyToTest(), dashboard_client, _ursim_action()]
    )


def generate_driver_test_description(
    tf_prefix="", controller_spawner_timeout=TIMEOUT_WAIT_SERVICE_INITIAL
):
    ur_type = LaunchConfiguration("ur_type")

    launch_arguments = {
        "robot_ip": "192.168.56.101",
        "ur_type": ur_type,
        "launch_rviz": "false",
        "controller_spawner_timeout": str(controller_spawner_timeout),
        "initial_joint_controller": "scaled_joint_trajectory_controller",
        "headless_mode": "true",
        "launch_dashboard_client": "true",
        "start_joint_controller": "false",
    }
    if tf_prefix:
        launch_arguments["tf_prefix"] = tf_prefix

    robot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments=launch_arguments.items(),
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
        _declare_launch_arguments()
        + [ReadyToTest(), wait_dashboard_server, _ursim_action(), driver_starter]
    )
