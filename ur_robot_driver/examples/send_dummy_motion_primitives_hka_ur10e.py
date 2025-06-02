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
from geometry_msgs.msg import PoseStamped
from industrial_robot_motion_interfaces.msg import MotionPrimitive, MotionArgument

joint_velocity = 1.0
joint_acceleration = 1.0
cart_velocity = 0.5
cart_acceleration = 0.5

# Joint movement to home position
msg_moveJ_1 = MotionPrimitive()
msg_moveJ_1.type = MotionPrimitive.LINEAR_JOINT
msg_moveJ_1.joint_positions = [1.57, -1.57, 1.57, -1.57, -1.57, -1.57]
msg_moveJ_1.blend_radius = 0.1
msg_moveJ_1.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=joint_velocity),
    MotionArgument(argument_name="acceleration", argument_value=joint_acceleration),
]  # MotionArgument(argument_name="move_time", argument_value=0.0),

# Linear movement down
msg_moveL_1 = MotionPrimitive()
msg_moveL_1.type = MotionPrimitive.LINEAR_CARTESIAN
msg_moveL_1.blend_radius = 0.05
msg_moveL_1.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=cart_velocity),
    MotionArgument(argument_name="acceleration", argument_value=cart_acceleration),
]
pose_L1 = PoseStamped()
pose_L1.pose.position.x = 0.174
pose_L1.pose.position.y = -0.692
pose_L1.pose.position.z = 0.1
pose_L1.pose.orientation.x = 1.0
pose_L1.pose.orientation.y = 0.0
pose_L1.pose.orientation.z = 0.0
pose_L1.pose.orientation.w = 0.0
msg_moveL_1.poses = [pose_L1]

# Linear movement up
msg_moveL_2 = MotionPrimitive()
msg_moveL_2.type = MotionPrimitive.LINEAR_CARTESIAN
msg_moveL_2.blend_radius = 0.05
msg_moveL_2.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=cart_velocity),
    MotionArgument(argument_name="acceleration", argument_value=cart_acceleration),
]
pose_L2 = PoseStamped()
pose_L2.pose.position.x = 0.174
pose_L2.pose.position.y = -0.692
pose_L2.pose.position.z = 0.5
pose_L2.pose.orientation.x = 1.0
pose_L2.pose.orientation.y = 0.0
pose_L2.pose.orientation.z = 0.0
pose_L2.pose.orientation.w = 0.0
msg_moveL_2.poses = [pose_L2]

# Joint movement
msg_moveJ_2 = MotionPrimitive()
msg_moveJ_2.type = MotionPrimitive.LINEAR_JOINT
msg_moveJ_2.blend_radius = 0.1
msg_moveJ_2.joint_positions = [0.9, -1.57, 1.57, -1.57, -1.57, -1.57]  # xyz = 0.294, 0.650, 0.677
msg_moveJ_2.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=joint_velocity),
    MotionArgument(argument_name="acceleration", argument_value=joint_acceleration),
]

# Circular movement
msg_moveC_1 = MotionPrimitive()
msg_moveC_1.type = MotionPrimitive.CIRCULAR_CARTESIAN
msg_moveC_1.blend_radius = 0.0
msg_moveC_1.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=cart_velocity),
    MotionArgument(argument_name="acceleration", argument_value=cart_acceleration),
]
pose_C1_via = PoseStamped()
pose_C1_via.pose.position.x = 0.174
pose_C1_via.pose.position.y = -0.9
pose_C1_via.pose.position.z = 0.5
pose_C1_via.pose.orientation.x = 1.0
pose_C1_via.pose.orientation.y = 0.0
pose_C1_via.pose.orientation.z = 0.0
pose_C1_via.pose.orientation.w = 0.0
pose_C1_goal = PoseStamped()
pose_C1_goal.pose.position.x = 0.5
pose_C1_goal.pose.position.y = -0.692
pose_C1_goal.pose.position.z = 0.5
pose_C1_goal.pose.orientation.x = 1.0
pose_C1_goal.pose.orientation.y = 0.0
pose_C1_goal.pose.orientation.z = 0.0
pose_C1_goal.pose.orientation.w = 0.0
msg_moveC_1.poses = [pose_C1_goal, pose_C1_via]  # first pose is goal, second is via point

msg_start_sequence = MotionPrimitive()
msg_start_sequence.type = MotionPrimitive.MOTION_SEQUENCE_START

msg_end_sequence = MotionPrimitive()
msg_end_sequence.type = MotionPrimitive.MOTION_SEQUENCE_END


msg_moveL_3 = MotionPrimitive()
msg_moveL_3.type = MotionPrimitive.LINEAR_CARTESIAN
msg_moveL_3.blend_radius = 0.05
msg_moveL_3.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=cart_velocity),
    MotionArgument(argument_name="acceleration", argument_value=cart_acceleration),
]
msg_moveL_3.poses = [pose_C1_goal]


class MotionPublisher(Node):
    def __init__(self):
        super().__init__("motion_publisher")

        self.publisher_ = self.create_publisher(
            MotionPrimitive, "/motion_primitive_controller/reference", 10
        )

        # self.messages = [msg_moveJ_1, msg_moveL_1, msg_moveL_2, msg_moveJ_2, msg_moveC_1]
        # self.messages = [msg_start_sequence, msg_moveJ_1, msg_moveL_1, msg_moveL_2, msg_moveJ_2, msg_moveC_1, msg_end_sequence]
        self.messages = [
            msg_moveJ_1,
            msg_moveL_1,
            msg_moveL_2,
            msg_moveJ_2,
            msg_moveC_1,
            msg_start_sequence,
            msg_moveJ_1,
            msg_moveL_1,
            msg_moveL_2,
            msg_moveJ_2,
            msg_moveC_1,
            msg_end_sequence,
        ]

        self.get_logger().info(f"Number of messages: {len(self.messages)}")

        self.send_all_messages()

    def send_all_messages(self):
        for i, msg in enumerate(self.messages):
            self.get_logger().info(f"Sending message {i + 1} of {len(self.messages)}")
            self.publisher_.publish(msg)
            self.get_logger().info(f"Sent message {i + 1}: {msg}")


def main(args=None):
    rclpy.init(args=args)
    node = MotionPublisher()
    rclpy.spin_once(node, timeout_sec=1)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
