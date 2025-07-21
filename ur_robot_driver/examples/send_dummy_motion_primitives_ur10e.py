#!/usr/bin/env python3

# Copyright (c) 2025, b»robotized
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
#
# Authors: Mathias Fuhrer

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from control_msgs.msg import MotionPrimitive, MotionArgument, MotionPrimitiveSequence
from control_msgs.action import ExecuteMotionPrimitiveSequence
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalStatus
import threading
import sys

joint_velocity = 1.0
joint_acceleration = 1.0
cart_velocity = 0.5
cart_acceleration = 0.5
move_time = 0.0  # if move_time=0 vel and acc are used, otherwise move_time is used

# Joint movement to home position
moveJ_1 = MotionPrimitive()
moveJ_1.type = MotionPrimitive.LINEAR_JOINT
moveJ_1.joint_positions = [1.57, -1.57, 1.57, -1.57, -1.57, -1.57]
moveJ_1.blend_radius = 0.1
moveJ_1.additional_arguments = [
    MotionArgument(name="velocity", value=joint_velocity),
    MotionArgument(name="acceleration", value=joint_acceleration),
    MotionArgument(name="move_time", value=move_time),
]
# Linear movement down
moveL_1 = MotionPrimitive()
moveL_1.type = MotionPrimitive.LINEAR_CARTESIAN
moveL_1.blend_radius = 0.05
moveL_1.additional_arguments = [
    MotionArgument(name="velocity", value=cart_velocity),
    MotionArgument(name="acceleration", value=cart_acceleration),
]
pose_L1 = PoseStamped()
pose_L1.pose.position.x = 0.174
pose_L1.pose.position.y = -0.692
pose_L1.pose.position.z = 0.3
pose_L1.pose.orientation.x = 1.0
pose_L1.pose.orientation.y = 0.0
pose_L1.pose.orientation.z = 0.0
pose_L1.pose.orientation.w = 0.0
moveL_1.poses = [pose_L1]

# Linear movement up
moveL_2 = MotionPrimitive()
moveL_2.type = MotionPrimitive.LINEAR_CARTESIAN
moveL_2.blend_radius = 0.05
moveL_2.additional_arguments = moveL_1.additional_arguments.copy()
pose_L2 = PoseStamped()
pose_L2.pose.position.x = 0.174
pose_L2.pose.position.y = -0.692
pose_L2.pose.position.z = 0.7
pose_L2.pose.orientation.x = 1.0
pose_L2.pose.orientation.y = 0.0
pose_L2.pose.orientation.z = 0.0
pose_L2.pose.orientation.w = 0.0
moveL_2.poses = [pose_L2]

# Joint movement
moveJ_2 = MotionPrimitive()
moveJ_2.type = MotionPrimitive.LINEAR_JOINT
moveJ_2.blend_radius = 0.1
moveJ_2.joint_positions = [0.9, -1.57, 1.57, -1.57, -1.57, -1.57]
moveJ_2.additional_arguments = moveJ_1.additional_arguments.copy()

# Circular movement
moveC_1 = MotionPrimitive()
moveC_1.type = MotionPrimitive.CIRCULAR_CARTESIAN
moveC_1.blend_radius = 0.0
moveC_1.additional_arguments = moveL_1.additional_arguments.copy()
pose_C1_via = PoseStamped()
pose_C1_via.pose.position.x = 0.174
pose_C1_via.pose.position.y = -0.9
pose_C1_via.pose.position.z = 0.7
pose_C1_via.pose.orientation.x = 1.0
pose_C1_via.pose.orientation.y = 0.0
pose_C1_via.pose.orientation.z = 0.0
pose_C1_via.pose.orientation.w = 0.0
pose_C1_goal = PoseStamped()
pose_C1_goal.pose.position.x = 0.5
pose_C1_goal.pose.position.y = -0.692
pose_C1_goal.pose.position.z = 0.7
pose_C1_goal.pose.orientation.x = 1.0
pose_C1_goal.pose.orientation.y = 0.0
pose_C1_goal.pose.orientation.z = 0.0
pose_C1_goal.pose.orientation.w = 0.0
moveC_1.poses = [pose_C1_goal, pose_C1_via]  # first pose is goal, second is via point

# Motions to compare moprim and jtc mode (send_joint_positions.py for jtc mode)
eval_blend_radius = 0.0
eval_move_time = 1.0

moveJ_eval_0 = MotionPrimitive()
moveJ_eval_0.type = MotionPrimitive.LINEAR_JOINT
moveJ_eval_0.joint_positions = [1.57, -1.57, 1.57, -1.57, -1.57, -1.57]
moveJ_eval_0.blend_radius = eval_blend_radius
moveJ_eval_0.additional_arguments = [
    MotionArgument(name="move_time", value=eval_move_time),
]
moveJ_eval_1 = MotionPrimitive()
moveJ_eval_1.type = MotionPrimitive.LINEAR_JOINT
moveJ_eval_1.joint_positions = [1.57, -1.1, 1.0, -1.57, -1.57, -1.57]
moveJ_eval_1.blend_radius = eval_blend_radius
moveJ_eval_1.additional_arguments = [
    MotionArgument(name="move_time", value=eval_move_time),
]
moveJ_eval_2 = MotionPrimitive()
moveJ_eval_2.type = MotionPrimitive.LINEAR_JOINT
moveJ_eval_2.joint_positions = [2.0, -0.9, 0.7, -1.57, -1.57, -1.57]
moveJ_eval_2.blend_radius = eval_blend_radius
moveJ_eval_2.additional_arguments = [
    MotionArgument(name="move_time", value=eval_move_time),
]
moveJ_eval_3 = MotionPrimitive()
moveJ_eval_3.type = MotionPrimitive.LINEAR_JOINT
moveJ_eval_3.joint_positions = [2.4, -1.57, 1.57, -1.57, -1.57, -1.57]
moveJ_eval_3.blend_radius = eval_blend_radius
moveJ_eval_3.additional_arguments = [
    MotionArgument(name="move_time", value=eval_move_time),
]
moveJ_eval_4 = MotionPrimitive()
moveJ_eval_4.type = MotionPrimitive.LINEAR_JOINT
moveJ_eval_4.joint_positions = [1.57, -1.57, 1.57, -1.57, -1.57, -1.57]
moveJ_eval_4.blend_radius = eval_blend_radius
moveJ_eval_4.additional_arguments = [
    MotionArgument(name="move_time", value=eval_move_time),
]
moveJ_eval_5 = MotionPrimitive()
moveJ_eval_5.type = MotionPrimitive.LINEAR_JOINT
moveJ_eval_5.joint_positions = [1.57, -1.1, 1.0, -1.57, -1.57, -1.57]
moveJ_eval_5.blend_radius = eval_blend_radius
moveJ_eval_5.additional_arguments = [
    MotionArgument(name="move_time", value=eval_move_time),
]
moveJ_eval_6 = MotionPrimitive()
moveJ_eval_6.type = MotionPrimitive.LINEAR_JOINT
moveJ_eval_6.joint_positions = [1.1, -0.9, 0.7, -1.57, -1.57, -1.57]
moveJ_eval_6.blend_radius = eval_blend_radius
moveJ_eval_6.additional_arguments = [
    MotionArgument(name="move_time", value=eval_move_time),
]
moveJ_eval_7 = MotionPrimitive()
moveJ_eval_7.type = MotionPrimitive.LINEAR_JOINT
moveJ_eval_7.joint_positions = [0.7, -1.57, 1.57, -1.57, -1.57, -1.57]
moveJ_eval_7.blend_radius = eval_blend_radius
moveJ_eval_7.additional_arguments = [
    MotionArgument(name="move_time", value=eval_move_time),
]
moveJ_eval_8 = MotionPrimitive()
moveJ_eval_8.type = MotionPrimitive.LINEAR_JOINT
moveJ_eval_8.joint_positions = [1.57, -1.57, 1.57, -1.57, -1.57, -1.57]
moveJ_eval_8.blend_radius = eval_blend_radius
moveJ_eval_8.additional_arguments = [
    MotionArgument(name="move_time", value=eval_move_time),
]


class ExecuteMotionClient(Node):
    def __init__(self):
        super().__init__("motion_sequence_client")

        # Initialize action client for ExecuteMotionPrimitiveSequence action
        self._client = ActionClient(
            self,
            ExecuteMotionPrimitiveSequence,
            "/motion_primitive_forward_controller/motion_sequence",
        )

        # Initialize client for cancel_goal service
        self._cancel_client = self.create_client(
            CancelGoal, "/motion_primitive_forward_controller/motion_sequence/_action/cancel_goal"
        )

        self._goal_id = None  # To store the goal ID for cancellation

        # Send the motion goal
        self._send_goal()

        # Start a thread to listen for ENTER key press to cancel the goal
        thread = threading.Thread(target=self._wait_for_keypress, daemon=True)
        thread.start()

    def _send_goal(self):
        """Send the motion sequence goal to the action server."""
        self.get_logger().info("Waiting for action server...")
        self._client.wait_for_server()

        goal_msg = ExecuteMotionPrimitiveSequence.Goal()
        goal_msg.trajectory = MotionPrimitiveSequence()

        # "pick" sequence with moveC in the end
        goal_msg.trajectory.motions = [moveJ_1, moveL_1, moveL_2, moveJ_2, moveC_1]

        # evaluation sequence with moveJ movements
        # goal_msg.trajectory.motions = [
        #     moveJ_eval_0,
        #     moveJ_eval_1,
        #     moveJ_eval_2,
        #     moveJ_eval_3,
        #     moveJ_eval_4,
        #     moveJ_eval_5,
        #     moveJ_eval_6,
        #     moveJ_eval_7,
        #     moveJ_eval_8,
        # ]

        self.get_logger().info(
            f"Sending {len(goal_msg.trajectory.motions)} motion primitives as a sequence..."
        )
        send_goal_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback called when the action server accepts or rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        self._goal_id = goal_handle.goal_id  # Store goal ID for cancellation

        # Wait for result asynchronously
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Receive feedback about the current motion primitive being executed."""
        current_index = feedback_msg.feedback.current_primitive_index
        self.get_logger().info(f"Executing primitive index: {current_index}")

    def result_callback(self, future):
        """Handle the result from the action server after goal finishes or is canceled."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Motion sequence executed successfully!")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Motion sequence was canceled.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Motion sequence execution failed.")
        else:
            self.get_logger().error(f"Execution ended with status: {status}")

        rclpy.shutdown()

    def _wait_for_keypress(self):
        """Wait for the user to press ENTER key to cancel the motion sequence."""
        print("Press ENTER to cancel the motion sequence...")
        while True:
            input_str = sys.stdin.readline().strip()
            if input_str == "":
                self.get_logger().info("ENTER key pressed: sending cancel request.")
                self.cancel_goal()
                break

    def cancel_goal(self):
        """Send a cancel request for the currently running goal."""
        if self._goal_id is None:
            self.get_logger().warn("No goal to cancel (goal_id not set yet).")
            return

        if not self._cancel_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Cancel service is not available.")
            return

        request = CancelGoal.Request()
        request.goal_info.goal_id = self._goal_id

        future = self._cancel_client.call_async(request)
        future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future):
        """Handle the response from the cancel service call."""
        try:
            response = future.result()
            if response.return_code == 0:
                self.get_logger().info("Cancel request accepted.")
            elif response.return_code == 1:
                self.get_logger().warn("Cancel request rejected.")
            else:
                self.get_logger().warn(f"Cancel returned code: {response.return_code}")
        except Exception as e:
            self.get_logger().error(f"Failed to call cancel service: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ExecuteMotionClient()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
