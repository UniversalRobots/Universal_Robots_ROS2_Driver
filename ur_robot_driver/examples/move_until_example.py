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

import time

import rclpy
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_msgs.action import FollowJointTrajectoryUntil

ROBOT_JOINTS = [
    "elbow_joint",
    "shoulder_lift_joint",
    "shoulder_pan_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class MoveUntilExample(rclpy.node.Node):
    def __init__(self):
        super().__init__("move_until_example")

        self._send_goal_future = None
        self._get_result_future = None
        self._goal_handle = None
        self._action_client = ActionClient(
            self, FollowJointTrajectoryUntil, "/trajectory_until_node/execute"
        )
        self._action_client.wait_for_server()
        self.test_traj = {
            "waypts": [[1.5, -1.5, 0.0, -1.5, -1.5, -1.5], [2.1, -1.2, 0.0, -2.4, -1.5, -1.5]],
            "time_vec": [Duration(sec=3, nanosec=0), Duration(sec=6, nanosec=0)],
        }
        self.get_logger().info("Initialized")

    def cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info("Cancelling goal")
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done)

    def cancel_done(self, future):
        try:
            future.result()
            self.get_logger().info("Goal cancelled successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to cancel goal: {e}")

    def process(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = ROBOT_JOINTS

        trajectory.points = [
            JointTrajectoryPoint(
                positions=self.test_traj["waypts"][i], time_from_start=self.test_traj["time_vec"][i]
            )
            for i in range(len(self.test_traj["waypts"]))
        ]
        goal = FollowJointTrajectoryUntil.Goal(
            trajectory=trajectory, until_type=FollowJointTrajectoryUntil.Goal.TOOL_CONTACT
        )
        self._send_goal_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, self._send_goal_future)
        self._goal_handle = self._send_goal_future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            raise RuntimeError("Goal rejected :(")

        self.get_logger().info(
            f"Goal accepted with ID: {bytes(self._goal_handle.goal_id.uuid).hex()}\n"
        )

        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        print(result)
        if result is None:
            self.get_logger().error("Result is None")
            return
        if result.error_code != FollowJointTrajectoryUntil.Result.SUCCESSFUL:
            self.get_logger().error(f"Result error code: {result.error_code}")
            return
        if result.until_condition_result == FollowJointTrajectoryUntil.Result.NOT_TRIGGERED:
            self.get_logger().info("Trajectory executed without tool contact trigger")
        else:
            self.get_logger().info("Trajectory executed with tool contact trigger")

        self.get_logger().info("Trajectory executed successfully with tool contact condition")


if __name__ == "__main__":
    rclpy.init()

    node = MoveUntilExample()
    try:
        node.process()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")
        node.cancel_goal()
        time.sleep(2)

    rclpy.shutdown()
