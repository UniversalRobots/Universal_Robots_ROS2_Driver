# Copyright (c) 2021 PickNik, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Vatan Aksoy Tezer
#
# Description: A follow joint trajectory action server as a replacement for ros2_control to control a robot with MoveIt in Ignition Gazebo

import rclpy
from rclpy.action import ActionServer, server
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory


class FollowJointTrajectoryActionServer(Node):
    def __init__(self):
        super().__init__("follow_joint_trajectory_action_server")
        print("Starting UR Ignition action server")
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory, "/joint_trajectory", 10
        )
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
            self.execute_callback,
        )

    def execute_callback(self, goal_handle: server.ServerGoalHandle):
        self.get_logger().info("Executing goal...")
        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory  # type: JointTrajectory
        self.joint_trajectory_publisher.publish(trajectory)
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    follow_joint_trajectory_action_server = FollowJointTrajectoryActionServer()
    rclpy.spin(follow_joint_trajectory_action_server)


if __name__ == "__main__":
    main()
