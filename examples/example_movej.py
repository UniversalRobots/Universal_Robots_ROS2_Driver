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
from control_msgs.msg import JointTolerance

from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory as JTmsg, JointTrajectoryPoint
from ur_msgs.srv import SetIO
from controller_manager_msgs.srv import (
    UnloadController,
    LoadController,
    ConfigureController,
    SwitchController,
    ListControllers,
)
import time

TIMEOUT_WAIT_SERVICE = 10
TIMEOUT_WAIT_SERVICE_INITIAL = 60
TIMEOUT_WAIT_ACTION = 10

ROBOT_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


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
        service_interfaces = {
            "/io_and_status_controller/set_io": SetIO,
            "/controller_manager/load_controller": LoadController,
            "/controller_manager/unload_controller": UnloadController,
            "/controller_manager/configure_controller": ConfigureController,
            "/controller_manager/switch_controller": SwitchController,
            "/controller_manager/list_controllers": ListControllers,
        }
        self.service_clients = {
            srv_name: waitForService(self.node, srv_name, srv_type)
            for (srv_name, srv_type) in service_interfaces.items()
        }
        self.jtc_action_client = waitForAction(
            self.node,
            "/passthrough_trajectory_controller/forward_joint_trajectory",
            FollowJointTrajectory,
        )
        time.sleep(2)

    def set_io(self, pin, value):
        """Test to set an IO."""
        set_io_req = SetIO.Request()
        set_io_req.fun = 1
        set_io_req.pin = pin
        set_io_req.state = value

        self.call_service("/io_and_status_controller/set_io", set_io_req)

    def send_trajectory(self, waypts, time_vec):
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

        tolerances = [JointTolerance(position=0.001) for i in range(6)]
        time_tolerance = Duration()
        time_tolerance.sec = 1
        # Sending trajectory goal
        goal_response = self.call_action(
            self.jtc_action_client,
            FollowJointTrajectory.Goal(
                trajectory=joint_trajectory,
                goal_tolerance=tolerances,
                goal_time_tolerance=time_tolerance,
            ),
        )
        if goal_response.accepted is False:
            raise Exception("trajectory was not accepted")

        # Verify execution

        result = self.get_result(self.jtc_action_client, goal_response)
        return result

    def call_service(self, srv_name, request):
        future = self.service_clients[srv_name].call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            return future.result()
        else:
            raise Exception(f"Exception while calling service: {future.exception()}")

    def call_action(self, ac_client, g):
        future = ac_client.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            return future.result()
        else:
            raise Exception(f"Exception while calling action: {future.exception()}")

    def get_result(self, ac_client, goal_response):
        future_res = ac_client._get_result_async(goal_response)
        rclpy.spin_until_future_complete(self.node, future_res)
        if future_res.result() is not None:
            return future_res.result().result
        else:
            raise Exception(f"Exception while calling action: {future_res.exception()}")

    def load_passthrough_controller(self):
        list_request = ListControllers.Request()
        list_response = robot.call_service("/controller_manager/list_controllers", list_request)
        names = []
        # Find loaded controllers
        for controller in list_response.controller:
            names.append(controller.name)
        # Check whether the passthrough controller is already loaded
        try:
            names.index("passthrough_trajectory_controller")
        except ValueError:
            print("Loading controller")
            load_request = LoadController.Request()
            load_request.name = "passthrough_trajectory_controller"
            self.call_service("/controller_manager/load_controller", load_request)
            configure_request = ConfigureController.Request()
            configure_request.name = "passthrough_trajectory_controller"
            self.call_service("/controller_manager/configure_controller", configure_request)
            list_request = ListControllers.Request()
            list_response = robot.call_service("/controller_manager/list_controllers", list_request)
            names.clear()
            # Update the list of controller names.
            for controller in list_response.controller:
                names.append(controller.name)
        id = names.index("passthrough_trajectory_controller")
        switch_request = SwitchController.Request()
        # Check if passthrough controller is inactive, and activate it if it is.
        if list_response.controller[id].state == "inactive":
            switch_request.activate_controllers = ["passthrough_trajectory_controller"]
        # Check that the scaled joint trajectory controller is loaded and active, deactivate if it is.
        try:
            id = names.index("scaled_joint_trajectory_controller")
            if list_response.controller[id].state == "active":
                switch_request.deactivate_controllers = ["scaled_joint_trajectory_controller"]
        except ValueError:
            switch_request.deactivate_controllers = []
        finally:
            switch_request.strictness = 1  # Best effort switching, will not terminate program if controller is already running
            switch_request.activate_asap = False
            switch_request.timeout = Duration(sec=2, nanosec=0)
            self.call_service("/controller_manager/switch_controller", switch_request)
        # Try unloading the scaled joint trajectory controller
        try:
            names.index("scaled_joint_trajectory_controller")
            unload_request = UnloadController.Request()
            unload_request.name = "scaled_joint_trajectory_controller"
            self.call_service("/controller_manager/unload_controller", unload_request)
        except ValueError:
            print("scaled_joint_trajectory_controller not loaded, skipping unload")
        # Connect to the passthrough controller action
        finally:
            self.jtc_action_client = waitForAction(
                self.node,
                "/passthrough_trajectory_controller/forward_joint_trajectory",
                FollowJointTrajectory,
            )
            time.sleep(2)

    def switch_controller(self, active, inactive):
        switch_request = SwitchController.Request()
        # Check if passthrough controller is inactive, and activate it if it is.
        switch_request.activate_controllers = [active]
        # Check that the scaled joint trajectory controller is loaded and active, deactivate if it is.
        switch_request.deactivate_controllers = [inactive]
        switch_request.strictness = (
            1  # Best effort switching, will not terminate program if controller is already running
        )
        switch_request.activate_asap = False
        switch_request.timeout = Duration(sec=2, nanosec=0)
        self.call_service("/controller_manager/switch_controller", switch_request)


if __name__ == "__main__":
    rclpy.init()
    node = Node("robot_driver_test")
    robot = Robot(node)

    # The following list are arbitrary joint positions, change according to your own needs
    waypts = [
        [-1.58, -1.692, -1.4311, -0.0174, 1.5882, 0.0349],
        [-3, -1.692, -1.4311, -0.0174, 1.5882, 0.0349],
        [-1.58, -1.692, -1.4311, -0.0174, 1.5882, 0.0349],
    ]
    time_vec = [Duration(sec=4, nanosec=0), Duration(sec=8, nanosec=0), Duration(sec=12, nanosec=0)]

    # Execute trajectory on robot, make sure that the robot is booted and the control script is running
    robot.send_trajectory(waypts, time_vec)
