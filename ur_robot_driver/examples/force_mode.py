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

import sys
import time

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs  # noqa # pylint: disable=unused-import

from builtin_interfaces.msg import Duration

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from visualization_msgs.msg import Marker

from geometry_msgs.msg import (
    Point,
    PoseStamped,
    Wrench,
    Vector3,
    Vector3Stamped,
)
from ur_msgs.srv import SetForceMode

from examples import Robot


class ForceModeExample(Node):
    def __init__(self):
        super().__init__("force_mode_example")

        self.robot = Robot(self)
        # Add force mode service to service interfaces and re-init robot
        self.robot.service_interfaces.update(
            {"/force_mode_controller/start_force_mode": SetForceMode}
        )
        self.robot.service_interfaces.update({"/force_mode_controller/stop_force_mode": Trigger})
        self.robot.init_robot()

        self.marker_publisher = self.create_publisher(Marker, "direction", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.direction = Vector3(x=0.0, y=0.0, z=1.0)

        self.marker_published = False
        self.force_mode_start_time = None
        self.startup()

        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(1.0, self.on_timer, self.timer_cb_group)

    def startup(self):
        # Press play on the robot
        self.robot.call_service("/dashboard_client/play", Trigger.Request())

        time.sleep(0.5)
        # Start controllers
        self.robot.call_service(
            "/controller_manager/switch_controller",
            SwitchController.Request(
                deactivate_controllers=[
                    "scaled_joint_trajectory_controller",
                    "forward_position_controller",
                ],
                activate_controllers=[
                    "passthrough_trajectory_controller",
                    "force_mode_controller",
                ],
                strictness=SwitchController.Request.BEST_EFFORT,
            ),
        )
        self.move_to_starting_pose()

    def on_timer(self):
        if not self.marker_published:
            self.publish_direction_marker()
            self.marker_published = True
        elif self.force_mode_start_time is None:
            self.start_force_mode()
            self.force_mode_start_time = self.get_clock().now()
            self.get_logger().info("Force mode started.")
        elif (self.get_clock().now() - self.force_mode_start_time).nanoseconds > 3e9:
            self.get_logger().info("Stopping force mode after 3 seconds.")
            self.robot.call_service("/force_mode_controller/stop_force_mode", Trigger.Request())
            sys.exit(0)

    def move_to_starting_pose(self):
        # Move robot in to position
        self.robot.send_trajectory(
            waypts=[[-1.6, -1.55, -1.7, -1.0, 2.05, 0.5]],
            time_vec=[Duration(sec=5, nanosec=0)],
            action_client=self.robot.passthrough_trajectory_action_client,
        )

    def start_force_mode(self):
        # Create task frame for force mode
        frame_stamp = PoseStamped()
        frame_stamp.header.frame_id = "tool0_controller"
        frame_stamp.pose.position.x = 0.0
        frame_stamp.pose.position.y = 0.0
        frame_stamp.pose.position.z = 0.0
        frame_stamp.pose.orientation.x = 0.0
        frame_stamp.pose.orientation.y = 0.0
        frame_stamp.pose.orientation.z = 0.0
        frame_stamp.pose.orientation.w = 1.0

        wrench_vec = Wrench(
            force=Vector3(x=0.0, y=0.0, z=10.0), torque=Vector3(x=0.0, y=0.0, z=0.0)
        )
        type_spec = SetForceMode.Request.NO_TRANSFORM

        # Specify max speeds and deviations of force mode
        speed_limits = Twist()
        speed_limits.linear = Vector3(x=0.0, y=0.0, z=1.0)
        speed_limits.angular = Vector3(x=0.0, y=0.0, z=0.0)
        deviation_limits = [0.005, 0.005, 0.005, 0.005, 0.005, 0.005]

        # specify damping and gain scaling
        damping_factor = 0.1
        gain_scale = 0.8

        req = SetForceMode.Request()
        req.task_frame = frame_stamp
        req.selection_vector_x = False
        req.selection_vector_y = False
        req.selection_vector_z = True
        req.selection_vector_rx = False
        req.selection_vector_ry = False
        req.selection_vector_rz = False
        req.wrench = wrench_vec
        req.type = type_spec
        req.speed_limits = speed_limits
        req.deviation_limits = deviation_limits
        req.damping_factor = damping_factor
        req.gain_scaling = gain_scale

        self.get_logger().info(f"Starting force mode with {req}")
        self.robot.call_service("/force_mode_controller/start_force_mode", req)

    def publish_direction_marker(self):
        """Publish a line strip going from the current TCP position to the desired direction."""
        to_frame_rel = "base"
        from_frame_rel = "tool0_controller"
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time(),
                timeout=rclpy.time.Duration(seconds=10.0),
            )
            self.get_logger().info(
                f"[{t.transform.translation.x}, {t.transform.translation.y}, {t.transform.translation.z}]"
            )
            direction_vec = Vector3Stamped()
            direction_vec.header.frame_id = from_frame_rel
            direction_vec.header.stamp = rclpy.time.Time()
            direction_vec.vector = self.direction
            transformed_direction = self.tf_buffer.transform(
                direction_vec, to_frame_rel, timeout=rclpy.time.Duration(seconds=1.0)
            )
        except TransformException as ex:
            self.get_logger().info(f"Could not transform {to_frame_rel} to {from_frame_rel}: {ex}")
            return

        marker = Marker()
        marker.header.frame_id = "base"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.ARROW
        marker.id = 0
        marker.action = marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.02
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points.append(
            Point(
                x=t.transform.translation.x,
                y=t.transform.translation.y,
                z=t.transform.translation.z,
            )
        )
        marker.points.append(
            Point(
                x=t.transform.translation.x + transformed_direction.vector.x,
                y=t.transform.translation.y + transformed_direction.vector.y,
                z=t.transform.translation.z + transformed_direction.vector.z,
            )
        )
        self.marker_publisher.publish(marker)


if __name__ == "__main__":
    rclpy.init()

    node = ForceModeExample()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    rclpy.shutdown()

    # time.sleep(0.5)

    # # Send request to controller
