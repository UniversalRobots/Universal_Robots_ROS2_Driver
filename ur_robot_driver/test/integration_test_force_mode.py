#!/usr/bin/env python
# Copyright 2019, Universal Robots A/S
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

import os
import sys
import time
import unittest

import pytest

import launch_testing
import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import std_msgs
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    Quaternion,
    Point,
    Twist,
    Wrench,
    Vector3,
)

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    ControllerManagerInterface,
    DashboardInterface,
    ForceModeInterface,
    IoStatusInterface,
    ConfigurationInterface,
    generate_driver_test_description,
)

TIMEOUT_EXECUTE_TRAJECTORY = 30


def are_quaternions_same(q1, q2, tolerance):
    dot_product = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w
    return (abs(dot_product) - 1.0) < tolerance


@pytest.mark.launch_test
@launch_testing.parametrize(
    "tf_prefix",
    [(""), ("my_ur_")],
)
def generate_test_description(tf_prefix):
    return generate_driver_test_description(tf_prefix=tf_prefix)


class RobotDriverTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = Node("robot_driver_test")
        time.sleep(1)
        cls.init_robot(cls)

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        cls.node.destroy_node()
        rclpy.shutdown()

    def init_robot(self):
        self._dashboard_interface = DashboardInterface(self.node)
        self._controller_manager_interface = ControllerManagerInterface(self.node)
        self._io_status_controller_interface = IoStatusInterface(self.node)
        self._configuration_controller_interface = ConfigurationInterface(self.node)

    def setUp(self):
        self._dashboard_interface.start_robot()
        time.sleep(1)
        self.assertTrue(self._io_status_controller_interface.resend_robot_program().success)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def lookup_tcp_in_base(self, tf_prefix, timepoint):
        trans = None
        while not trans:
            rclpy.spin_once(self.node)
            try:
                trans = self.tf_buffer.lookup_transform(
                    tf_prefix + "base", tf_prefix + "tool0", timepoint
                )
            except TransformException:
                pass
        return trans

    def test_force_mode_controller(self, tf_prefix):
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=[
                    "force_mode_controller",
                ],
                deactivate_controllers=[
                    "scaled_joint_trajectory_controller",
                    "joint_trajectory_controller",
                ],
            ).ok
        )
        self._force_mode_controller_interface = ForceModeInterface(self.node)

        # Create task frame for force mode
        point = Point(x=0.8, y=0.8, z=0.8)
        orientation = Quaternion(x=0.7071, y=0.0, z=0.0, w=0.7071)
        task_frame_pose = Pose()
        task_frame_pose.position = point
        task_frame_pose.orientation = orientation
        header = std_msgs.msg.Header(frame_id=tf_prefix + "base")
        header.stamp.sec = int(time.time()) + 1
        header.stamp.nanosec = 0
        frame_stamp = PoseStamped()
        frame_stamp.header = header
        frame_stamp.pose = task_frame_pose

        # Create compliance vector (which axes should be force controlled)
        compliance = [False, False, True, False, False, False]

        # Create Wrench message for force mode
        wrench = Wrench()
        wrench.force = Vector3(x=0.0, y=0.0, z=5.0)
        wrench.torque = Vector3(x=0.0, y=0.0, z=0.0)

        # Specify interpretation of task frame (no transform)
        type_spec = 2

        # Specify max speeds and deviations of force mode
        speed_limits = Twist()
        speed_limits.linear = Vector3(x=0.0, y=0.0, z=1.0)
        speed_limits.angular = Vector3(x=0.0, y=0.0, z=1.0)
        deviation_limits = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

        # specify damping and gain scaling
        damping_factor = 0.1
        gain_scale = 0.8

        trans_before = self.lookup_tcp_in_base(tf_prefix, rclpy.time.Time())

        # Send request to controller
        res = self._force_mode_controller_interface.start_force_mode(
            task_frame=frame_stamp,
            selection_vector_x=compliance[0],
            selection_vector_y=compliance[1],
            selection_vector_z=compliance[2],
            selection_vector_rx=compliance[3],
            selection_vector_ry=compliance[4],
            selection_vector_rz=compliance[5],
            wrench=wrench,
            type=type_spec,
            speed_limits=speed_limits,
            deviation_limits=deviation_limits,
            damping_factor=damping_factor,
            gain_scaling=gain_scale,
        )
        self.assertTrue(res.success)

        time.sleep(5.0)

        trans_after = self.lookup_tcp_in_base(tf_prefix, self.node.get_clock().now())

        # task frame and wrench determines the expected motion
        # In the example we used
        #   - a task frame rotated pi/2 deg around the base frame's x axis
        #   - a wrench with a positive z component for the force
        # => we should expect a motion in negative y of the base frame
        self.assertTrue(trans_after.transform.translation.y < trans_before.transform.translation.y)
        self.assertAlmostEqual(
            trans_after.transform.translation.x,
            trans_before.transform.translation.x,
            delta=0.001,
        )
        self.assertAlmostEqual(
            trans_after.transform.translation.z,
            trans_before.transform.translation.z,
            delta=0.001,
        )
        self.assertTrue(
            are_quaternions_same(
                trans_after.transform.rotation, trans_before.transform.rotation, 0.001
            )
        )

        res = self._force_mode_controller_interface.stop_force_mode()
        self.assertTrue(res.success)

        # Deactivate controller
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                deactivate_controllers=["force_mode_controller"],
            ).ok
        )

    def test_illegal_force_mode_types(self, tf_prefix):
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=[
                    "force_mode_controller",
                ],
                deactivate_controllers=[
                    "scaled_joint_trajectory_controller",
                    "joint_trajectory_controller",
                ],
            ).ok
        )
        self._force_mode_controller_interface = ForceModeInterface(self.node)

        # Create task frame for force mode
        point = Point(x=0.0, y=0.0, z=0.0)
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        task_frame_pose = Pose()
        task_frame_pose.position = point
        task_frame_pose.orientation = orientation
        header = std_msgs.msg.Header(frame_id=tf_prefix + "base")
        header.stamp.sec = int(time.time()) + 1
        header.stamp.nanosec = 0
        frame_stamp = PoseStamped()
        frame_stamp.header = header
        frame_stamp.pose = task_frame_pose

        res = self._force_mode_controller_interface.start_force_mode(task_frame=frame_stamp, type=0)
        self.assertFalse(res.success)
        res = self._force_mode_controller_interface.start_force_mode(task_frame=frame_stamp, type=4)
        self.assertFalse(res.success)
        res = self._force_mode_controller_interface.start_force_mode(task_frame=frame_stamp, type=1)
        self.assertTrue(res.success)
        res = self._force_mode_controller_interface.stop_force_mode()
        res = self._force_mode_controller_interface.start_force_mode(task_frame=frame_stamp, type=2)
        self.assertTrue(res.success)
        res = self._force_mode_controller_interface.stop_force_mode()
        res = self._force_mode_controller_interface.start_force_mode(task_frame=frame_stamp, type=3)
        self.assertTrue(res.success)
        res = self._force_mode_controller_interface.stop_force_mode()

    def test_illegal_task_frame(self, tf_prefix):
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=[
                    "force_mode_controller",
                ],
                deactivate_controllers=[
                    "scaled_joint_trajectory_controller",
                    "joint_trajectory_controller",
                ],
            ).ok
        )
        self._force_mode_controller_interface = ForceModeInterface(self.node)

        # Create task frame for force mode
        point = Point(x=0.0, y=0.0, z=0.0)
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        task_frame_pose = Pose()
        task_frame_pose.position = point
        task_frame_pose.orientation = orientation
        header = std_msgs.msg.Header(frame_id=tf_prefix + "base")
        header.stamp.sec = int(time.time()) + 1
        header.stamp.nanosec = 0
        frame_stamp = PoseStamped()
        frame_stamp.header = header
        frame_stamp.pose = task_frame_pose

        # Illegal frame name produces error
        header.frame_id = "nonexisting6t54"
        res = self._force_mode_controller_interface.start_force_mode(
            task_frame=frame_stamp,
        )
        self.assertFalse(res.success)
        header.frame_id = "base"

        # Illegal quaternion produces error
        task_frame_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
        res = self._force_mode_controller_interface.start_force_mode(
            task_frame=frame_stamp,
        )
        self.assertFalse(res.success)

    def test_start_force_mode_on_inactive_controller_fails(self, tf_prefix):
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=[],
                deactivate_controllers=[
                    "force_mode_controller",
                    "scaled_joint_trajectory_controller",
                    "joint_trajectory_controller",
                ],
            ).ok
        )
        self._force_mode_controller_interface = ForceModeInterface(self.node)

        # Create task frame for force mode
        point = Point(x=0.0, y=0.0, z=0.0)
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        task_frame_pose = Pose()
        task_frame_pose.position = point
        task_frame_pose.orientation = orientation
        header = std_msgs.msg.Header(frame_id=tf_prefix + "base")
        header.stamp.sec = int(time.time()) + 1
        header.stamp.nanosec = 0
        frame_stamp = PoseStamped()
        frame_stamp.header = header
        frame_stamp.pose = task_frame_pose

        res = self._force_mode_controller_interface.start_force_mode(
            task_frame=frame_stamp,
        )
        self.assertFalse(res.success)

    def test_deactivating_controller_stops_force_mode(self, tf_prefix):
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=[
                    "force_mode_controller",
                ],
                deactivate_controllers=[
                    "scaled_joint_trajectory_controller",
                    "joint_trajectory_controller",
                ],
            ).ok
        )
        self._force_mode_controller_interface = ForceModeInterface(self.node)

        # Create task frame for force mode
        point = Point(x=0.0, y=0.0, z=0.0)
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        task_frame_pose = Pose()
        task_frame_pose.position = point
        task_frame_pose.orientation = orientation
        header = std_msgs.msg.Header(frame_id=tf_prefix + "base")
        header.stamp.sec = int(time.time()) + 1
        header.stamp.nanosec = 0
        frame_stamp = PoseStamped()
        frame_stamp.header = header
        frame_stamp.pose = task_frame_pose

        # Create compliance vector (which axes should be force controlled)
        compliance = [False, False, True, False, False, False]

        # Create Wrench message for force mode
        wrench = Wrench()
        wrench.force = Vector3(x=0.0, y=0.0, z=5.0)
        wrench.torque = Vector3(x=0.0, y=0.0, z=0.0)

        # Specify interpretation of task frame (no transform)
        type_spec = 2

        # Specify max speeds and deviations of force mode
        speed_limits = Twist()
        speed_limits.linear = Vector3(x=0.0, y=0.0, z=1.0)
        speed_limits.angular = Vector3(x=0.0, y=0.0, z=1.0)
        deviation_limits = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

        # specify damping and gain scaling
        damping_factor = 0.1
        gain_scale = 0.8

        res = self._force_mode_controller_interface.start_force_mode(
            task_frame=frame_stamp,
            selection_vector_x=compliance[0],
            selection_vector_y=compliance[1],
            selection_vector_z=compliance[2],
            selection_vector_rx=compliance[3],
            selection_vector_ry=compliance[4],
            selection_vector_rz=compliance[5],
            wrench=wrench,
            type=type_spec,
            speed_limits=speed_limits,
            deviation_limits=deviation_limits,
            damping_factor=damping_factor,
            gain_scaling=gain_scale,
        )
        self.assertTrue(res.success)

        time.sleep(0.5)

        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=[],
                deactivate_controllers=[
                    "force_mode_controller",
                    "scaled_joint_trajectory_controller",
                    "joint_trajectory_controller",
                ],
            ).ok
        )
        self._force_mode_controller_interface = ForceModeInterface(self.node)

        time.sleep(0.5)
        trans_before_wait = self.lookup_tcp_in_base(tf_prefix, self.node.get_clock().now())

        # Make sure the robot didn't move anymore
        time.sleep(0.5)
        trans_after_wait = self.lookup_tcp_in_base(tf_prefix, self.node.get_clock().now())

        self.assertAlmostEqual(
            trans_before_wait.transform.translation.z, trans_after_wait.transform.translation.z
        )

    def test_params_out_of_range_fails(self, tf_prefix):
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=[
                    "force_mode_controller",
                ],
                deactivate_controllers=[
                    "scaled_joint_trajectory_controller",
                    "joint_trajectory_controller",
                ],
            ).ok
        )
        self._force_mode_controller_interface = ForceModeInterface(self.node)

        # Create task frame for force mode
        point = Point(x=0.0, y=0.0, z=0.0)
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        task_frame_pose = Pose()
        task_frame_pose.position = point
        task_frame_pose.orientation = orientation
        header = std_msgs.msg.Header(frame_id=tf_prefix + "base")
        header.stamp.sec = int(time.time()) + 1
        header.stamp.nanosec = 0
        frame_stamp = PoseStamped()
        frame_stamp.header = header
        frame_stamp.pose = task_frame_pose

        res = self._force_mode_controller_interface.start_force_mode(
            task_frame=frame_stamp, gain_scaling=-0.1
        )
        self.assertFalse(res.success)

        res = self._force_mode_controller_interface.start_force_mode(
            task_frame=frame_stamp, gain_scaling=0.0
        )
        self.assertTrue(res.success)
        res = self._force_mode_controller_interface.stop_force_mode()
        self.assertTrue(res.success)

        res = self._force_mode_controller_interface.start_force_mode(
            task_frame=frame_stamp, gain_scaling=2.0
        )
        self.assertTrue(res.success)
        res = self._force_mode_controller_interface.stop_force_mode()
        self.assertTrue(res.success)

        res = self._force_mode_controller_interface.start_force_mode(
            task_frame=frame_stamp, gain_scaling=2.1
        )
        self.assertFalse(res.success)

        # damping factor
        res = self._force_mode_controller_interface.start_force_mode(
            task_frame=frame_stamp, damping_factor=-0.1
        )
        self.assertFalse(res.success)

        res = self._force_mode_controller_interface.start_force_mode(
            task_frame=frame_stamp, damping_factor=0.0
        )
        self.assertTrue(res.success)
        res = self._force_mode_controller_interface.stop_force_mode()
        self.assertTrue(res.success)

        res = self._force_mode_controller_interface.start_force_mode(
            task_frame=frame_stamp, damping_factor=1.0
        )
        self.assertTrue(res.success)
        res = self._force_mode_controller_interface.stop_force_mode()
        self.assertTrue(res.success)

        res = self._force_mode_controller_interface.start_force_mode(
            task_frame=frame_stamp, damping_factor=1.1
        )
        self.assertFalse(res.success)
