#!/usr/bin/env python3
# Copyright 2024, Universal Robots
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

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory

from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory as JTmsg, JointTrajectoryPoint
from ur_msgs.srv import SetIO, SetForceMode
from controller_manager_msgs.srv import (
    UnloadController,
    LoadController,
    ConfigureController,
    SwitchController,
    ListControllers,
)
from std_srvs.srv import Trigger
from enum import Enum
from collections import namedtuple

TIMEOUT_WAIT_SERVICE = 10
TIMEOUT_WAIT_SERVICE_INITIAL = 60
TIMEOUT_WAIT_ACTION = 10

MOTION_CONTROLLERS = ["passthrough_trajectory_controller", "scaled_joint_trajectory_controller"]

ROBOT_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
Action_tuple = namedtuple("Actions", ["name", "action_type"])


class Actions(Enum):
    PASSTHROUGH_TRAJECTORY = Action_tuple(
        "/passthrough_trajectory_controller/follow_joint_trajectory", FollowJointTrajectory
    )
    FOLLOW_TRAJECTORY = Action_tuple(
        "/scaled_joint_trajectory_controller/follow_joint_trajectory", FollowJointTrajectory
    )


Service_tuple = namedtuple("Services", ["name", "service_type"])


class Services(Enum):
    Set_IO = Service_tuple("/io_and_status_controller/set_io", SetIO)
    Load_Controller = Service_tuple("/controller_manager/load_controller", LoadController)
    Unload_Controller = Service_tuple("/controller_manager/unload_controller", UnloadController)
    Configure_Controller = Service_tuple(
        "/controller_manager/configure_controller", ConfigureController
    )
    Switch_Controller = Service_tuple("/controller_manager/switch_controller", SwitchController)
    List_Controllers = Service_tuple("/controller_manager/list_controllers", ListControllers)
    start_force_mode = Service_tuple("/force_mode_controller/start_force_mode", SetForceMode)
    stop_force_mode = Service_tuple("/force_mode_controller/stop_force_mode", Trigger)
    play = Service_tuple("/dashboard_client/play", Trigger)


# Helper functions
def waitForService(node, srv_name, srv_type, timeout=TIMEOUT_WAIT_SERVICE):
    client = node.create_client(srv_type, srv_name)
    if client.wait_for_service(timeout) is False:
        raise Exception(f"Could not reach service '{srv_name}' within timeout of {timeout}")

    node.get_logger().info(f"Successfully connected to service '{srv_name}'")
    return client


def waitForAction(node, action_name, action_type, timeout=TIMEOUT_WAIT_ACTION):
    client = ActionClient(node, action_type, action_name)
    if client.wait_for_server(timeout) is False:
        raise Exception(
            f"Could not reach action server '{action_name}' within timeout of {timeout}"
        )

    node.get_logger().info(f"Successfully connected to action '{action_name}'")
    return client


class Robot:
    def __init__(self, node):
        self.node = node
        self.init_robot()

    def init_robot(self):

        self.service_clients = {
            service.name: waitForService(self.node, service.value.name, service.value.service_type)
            for (service) in Services
        }

        self.action_clients = {
            action.name: waitForAction(self.node, action.value.name, action.value.action_type)
            for (action) in Actions
        }

    def add_service(self, dict_key: str, service_tuple: Service_tuple):
        self.service_clients.update(
            {dict_key: waitForService(self.node, service_tuple.name, service_tuple.service_type)}
        )

    def add_action(self, dict_key: str, action_tuple: Action_tuple):
        self.action_clients.update(
            {dict_key: waitForAction(self.node, action_tuple.name, action_tuple.action_type)}
        )

    def play(self):
        self.call_service(Services.play, Trigger.Request())

    def set_io(self, pin, value):
        """Test to set an IO."""
        set_io_req = SetIO.Request()
        set_io_req.fun = 1
        set_io_req.pin = pin
        set_io_req.state = value

        self.call_service(Services.Set_IO, set_io_req)

    def follow_trajectory(self, waypts: list[list[float]], time_vec: list[float]):
        # No other motion controllers can be active at the same time as the scaled joint controller
        self.switch_controllers(
            ["scaled_joint_trajectory_controller"],
            [
                "passthrough_trajectory_controller",
                "force_mode_controller",
                "freedrive_mode_controller",
            ],
        )
        """Send robot trajectory."""
        if len(waypts) != len(time_vec):
            raise Exception("waypoints vector and time vec should be same length")

        # Construct test trajectory
        joint_trajectory = JTmsg()
        joint_trajectory.joint_names = ROBOT_JOINTS
        for i in range(len(waypts)):
            point = JointTrajectoryPoint()
            point.positions = waypts[i]
            point.time_from_start = time_vec[i]
            joint_trajectory.points.append(point)

        # Sending trajectory goal
        goal_response = self.call_action(
            Actions.FOLLOW_TRAJECTORY,
            FollowJointTrajectory.Goal(trajectory=joint_trajectory),
        )
        if goal_response.accepted is False:
            raise Exception("trajectory was not accepted")

        # Verify execution
        result = self.get_result(Actions.FOLLOW_TRAJECTORY, goal_response)
        return result

    def passthrough_trajectory(
        self,
        waypts: list[list[float]],
        time_vec: list[float],
        vels: list[list[float]] = [],
        accels: list[list[float]] = [],
        goal_time_tolerance=Duration(sec=1),
    ):
        # The scaled joint controller can't be active at the same time as the passthrough controller
        self.switch_controllers(
            ["passthrough_trajectory_controller"], ["scaled_joint_trajectory_controller"]
        )
        """Send trajectory through the passthrough controller."""
        if len(waypts) != len(time_vec):
            raise Exception("waypoints vector and time vec should be same length.")
        trajectory = JTmsg()
        trajectory.joint_names = ROBOT_JOINTS
        for i in range(len(waypts)):
            point = JointTrajectoryPoint()
            point.positions = waypts[i]
            point.time_from_start = time_vec[i]
            if len(vels) != 0:
                point.velocities = vels[i]
            if len(accels) != 0:
                point.accelerations = accels[i]
            trajectory.points.append(point)
        goal_response = self.call_action(
            Actions.PASSTHROUGH_TRAJECTORY,
            FollowJointTrajectory.Goal(
                trajectory=trajectory, goal_time_tolerance=goal_time_tolerance
            ),
        )
        if goal_response.accepted is False:
            raise Exception("trajectory was not accepted")

        # Verify execution
        result = self.get_result(Actions.PASSTHROUGH_TRAJECTORY, goal_response)
        return result

    def call_service(self, Service: Services, request):
        future = self.service_clients[Service.name].call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            return future.result()
        else:
            raise Exception(f"Exception while calling service: {future.exception()}")

    def call_action(self, Action: Actions, g):
        future = self.action_clients[Action.name].send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            return future.result()
        else:
            raise Exception(f"Exception while calling action: {future.exception()}")

    def get_result(self, Action: Actions, goal_response):
        future_res = self.action_clients[Action.name]._get_result_async(goal_response)
        rclpy.spin_until_future_complete(self.node, future_res)
        if future_res.result() is not None:
            return future_res.result().result
        else:
            raise Exception(f"Exception while calling action: {future_res.exception()}")

    def load_controller(self, controller_name: str):
        list_response = self.call_service(Services.List_Controllers, ListControllers.Request())
        names = []
        # Find loaded controllers
        for controller in list_response.controller:
            names.append(controller.name)
        # Check whether the controller is already loaded
        try:
            names.index(controller_name)
        except ValueError:
            print("Loading controller")
            load_request = LoadController.Request(name=controller_name)
            self.call_service(Services.Load_Controller, load_request)
            configure_request = ConfigureController.Request(name=controller_name)
            self.call_service(Services.Configure_Controller, configure_request)
            list_response = self.call_service(Services.List_Controllers, ListControllers.Request())
            names.clear()
            # Update the list of controller names.
            for controller in list_response.controller:
                names.append(controller.name)
        else:
            print("Controller already loaded")
        finally:
            print(f"Currently loaded controllers: {names}")

    def switch_controllers(self, active: list[str] = [], inactive: list[str] = []) -> bool:
        switch_request = SwitchController.Request()
        switch_request.activate_controllers = active
        switch_request.deactivate_controllers = inactive
        switch_request.strictness = (
            SwitchController.Request.BEST_EFFORT
        )  # Best effort switching, will not terminate program if controller is already running
        switch_request.activate_asap = False
        switch_request.timeout = Duration(sec=2, nanosec=0)
        return self.call_service(Services.Switch_Controller, switch_request)

    def start_force_mode(self, req: SetForceMode.Request):
        return self.call_service(Services.start_force_mode, req)

    def stop_force_mode(self):
        return self.call_service(Services.stop_force_mode, Trigger.Request())


if __name__ == "__main__":
    rclpy.init()
    node = Node("robot_driver_test")
    print("Available actions:")
    for action in Actions:
        print("Action Enum name: ", action.name)
        print("Action Ros name: ", action.value.name)
        print("Action Ros type: ", action.value.action_type)
    print("Available services:")
    for service in Services:
        print("Service Enum name: ", service.name)
        print("Service Ros name: ", service.value.name)
        print("Service Ros type: ", service.value.service_type)

    # robot = Robot(node)
    # robot.switch_controllers(
    #     ["passthrough_trajectory_controller"], ["scaled_joint_trajectory_controller"]
    # )
    # robot.add_service(
    #     "Zero_sensor", Service_tuple("/io_and_status_controller/zero_ftsensor", Trigger)
    # )

    # # The following list are arbitrary joint positions, change according to your own needs
    # waypts = [
    #     [-1, -2.5998, -1.004, -2.676, -0.992, -1.5406],
    #     [-0.1, -2.6998, -1.104, -2.676, -0.992, -1.5406],
    #     [-1, -2.5998, -1.004, -2.676, -0.992, -1.5406],
    # ]
    # time_vec = [Duration(sec=4, nanosec=0), Duration(sec=8, nanosec=0), Duration(sec=12, nanosec=0)]

    # # Execute trajectory on robot, make sure that the robot is booted and the control script is running
    # robot.passthrough_trajectory(waypts, time_vec)
