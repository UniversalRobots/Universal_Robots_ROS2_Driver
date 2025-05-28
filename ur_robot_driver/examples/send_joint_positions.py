#!/usr/bin/env python3

# Copyright (c) 2025, b»robotized
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
# Authors: Mathias Fuhrer

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        time.sleep(1)  # Allow time for publisher to be ready

    def publish_trajectory_startponit(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        msg.points = [JointTrajectoryPoint(positions=[1.57, -1.57, 1.57, -1.57, -1.57, -1.57], velocities=[0.0]*6, time_from_start=rclpy.duration.Duration(seconds=3).to_msg())] # H-KA UR10e
        self.publisher_.publish(msg)
        self.get_logger().info('Trajectory startpoint published successfully.')

    def publish_trajectory_sequence(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

        # Define multiple trajectory points
        # H-KA UR10e
        points = [
            # JointTrajectoryPoint(positions=[1.57, -1.57, 1.57, -1.57, -1.57, -1.57], velocities=[0.0]*6, time_from_start=rclpy.duration.Duration(seconds=).to_msg()),
            JointTrajectoryPoint(positions=[1.57, -1.1, 1.0, -1.57, -1.57, -1.57], velocities=[0.0]*6, time_from_start=rclpy.duration.Duration(seconds=1).to_msg()),
            JointTrajectoryPoint(positions=[2.0, -0.9, 0.7, -1.57, -1.57, -1.57], velocities=[0.0]*6, time_from_start=rclpy.duration.Duration(seconds=2).to_msg()),
            JointTrajectoryPoint(positions=[2.4, -1.57, 1.57, -1.57, -1.57, -1.57], velocities=[0.0]*6, time_from_start=rclpy.duration.Duration(seconds=3).to_msg()),
            JointTrajectoryPoint(positions=[1.57, -1.57, 1.57, -1.57, -1.57, -1.57], velocities=[0.0]*6, time_from_start=rclpy.duration.Duration(seconds=4).to_msg()),
            JointTrajectoryPoint(positions=[1.57, -1.1, 1.0, -1.57, -1.57, -1.57], velocities=[0.0]*6, time_from_start=rclpy.duration.Duration(seconds=5).to_msg()),
            JointTrajectoryPoint(positions=[1.1, -0.9, 0.7, -1.57, -1.57, -1.57], velocities=[0.0]*6, time_from_start=rclpy.duration.Duration(seconds=6).to_msg()),
            JointTrajectoryPoint(positions=[0.7, -1.57, 1.57, -1.57, -1.57, -1.57], velocities=[0.0]*6, time_from_start=rclpy.duration.Duration(seconds=7).to_msg()),
            JointTrajectoryPoint(positions=[1.57, -1.57, 1.57, -1.57, -1.57, -1.57], velocities=[0.0]*6, time_from_start=rclpy.duration.Duration(seconds=8).to_msg()),
        ]

        msg.points = points
        self.publisher_.publish(msg)
        self.get_logger().info('Trajectory sequence published successfully.')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    node.publish_trajectory_startponit()
    time.sleep(4)
    for _ in range(1):
        node.publish_trajectory_sequence()
        time.sleep(8)   # Adjust time to sequence length
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()