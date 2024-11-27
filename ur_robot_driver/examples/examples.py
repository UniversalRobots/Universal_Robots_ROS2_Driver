#!/usr/bin/env python3
# Copyright 2024, Universal Robots A/S
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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_msgs.srv import SetIO
from controller_manager_msgs.srv import SwitchController
from std_srvs.srv import Trigger

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
        self.service_interfaces = {
            "/io_and_status_controller/set_io": SetIO,
            "/dashboard_client/play": Trigger,
            "/controller_manager/switch_controller": SwitchController,
        }
        self.init_robot()

    def init_robot(self):
        self.service_clients = {
            srv_name: waitForService(self.node, srv_name, srv_type)
            for (srv_name, srv_type) in self.service_interfaces.items()
        }

        self.jtc_action_client = waitForAction(
            self.node,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectory,
        )
        self.passthrough_trajectory_action_client = waitForAction(
            self.node,
            "/passthrough_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectory,
        )

    def set_io(self, pin, value):
        """Test to set an IO."""
        set_io_req = SetIO.Request()
        set_io_req.fun = 1
        set_io_req.pin = pin
        set_io_req.state = value

        self.call_service("/io_and_status_controller/set_io", set_io_req)

    def send_trajectory(self, waypts, time_vec, action_client):
        """Send robot trajectory."""
        if len(waypts) != len(time_vec):
            raise Exception("waypoints vector and time vec should be same length")

        # Construct test trajectory
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = ROBOT_JOINTS
        for i in range(len(waypts)):
            point = JointTrajectoryPoint()
            point.positions = waypts[i]
            point.time_from_start = time_vec[i]
            joint_trajectory.points.append(point)

        # Sending trajectory goal
        goal_response = self.call_action(
            action_client, FollowJointTrajectory.Goal(trajectory=joint_trajectory)
        )
        if not goal_response.accepted:
            raise Exception("trajectory was not accepted")

        # Verify execution
        result = self.get_result(action_client, goal_response)
        return result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

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


if __name__ == "__main__":
    rclpy.init()
    node = Node("robot_driver_test")
    robot = Robot(node)

    # The following list are arbitrary joint positions, change according to your own needs
    waypts = [
        [-1.6006, -1.7272, -2.2030, -0.8079, 1.5951, -0.0311],
        [-1.2, -1.4, -1.9, -1.2, 1.5951, -0.0311],
        [-1.6006, -1.7272, -2.2030, -0.8079, 1.5951, -0.0311],
    ]
    time_vec = [Duration(sec=4, nanosec=0), Duration(sec=8, nanosec=0), Duration(sec=12, nanosec=0)]

    # Execute trajectory on robot, make sure that the robot is booted and the control script is running
    robot.send_trajectory(waypts, time_vec)

    # Set digital output 1 to true
    robot.set_io(1, 1.0)
