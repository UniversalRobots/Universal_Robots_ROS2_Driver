#!/usr/bin/env python3
# Copyright 2019, FZI Forschungszentrum Informatik
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
import logging
import os
import sys
import time
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_msgs.msg import IOStates

sys.path.append(os.path.dirname(__file__))
from test_common import ROBOT_JOINTS, TIMEOUT_EXECUTE_TRAJECTORY  # noqa: E402


def test_get_robot_software_version(robot):
    if not robot.mock_hardware:
        assert robot._configuration_controller_interface.get_robot_software_version().major != 0
    else:
        assert robot._configuration_controller_interface.get_robot_software_version().major == 0


def test_start_scaled_jtc_controller(robot):
    assert robot._controller_manager_interface.switch_controller(
        strictness=SwitchController.Request.BEST_EFFORT,
        activate_controllers=["scaled_joint_trajectory_controller"],
    ).ok


def test_start_passthrough_controller(robot):
    assert robot._controller_manager_interface.switch_controller(
        strictness=SwitchController.Request.BEST_EFFORT,
        activate_controllers=["passthrough_trajectory_controller"],
        deactivate_controllers=["scaled_joint_trajectory_controller"],
    ).ok
    assert robot._controller_manager_interface.switch_controller(
        strictness=SwitchController.Request.BEST_EFFORT,
        deactivate_controllers=["passthrough_trajectory_controller"],
        activate_controllers=["scaled_joint_trajectory_controller"],
    ).ok


def test_set_io(robot):
    """Test to set an IO and check whether it has been set."""
    # Create io callback to verify result
    if robot.mock_hardware:
        return True
    io_msg = None

    def io_msg_cb(msg):
        nonlocal io_msg
        io_msg = msg

    io_states_sub = robot.node.create_subscription(
        IOStates,
        "/io_and_status_controller/io_states",
        io_msg_cb,
        rclpy.qos.qos_profile_system_default,
    )

    # Set pin 0 to 1.0
    test_pin = 0

    logging.info("Setting pin %d to 1.0", test_pin)
    robot._io_status_controller_interface.set_io(fun=1, pin=test_pin, state=1.0)

    # Wait until the pin state has changed
    pin_state = False
    end_time = time.time() + 5
    while not pin_state and time.time() < end_time:
        rclpy.spin_once(robot.node, timeout_sec=0.1)
        if io_msg is not None:
            pin_state = io_msg.digital_out_states[test_pin].state

    assert pin_state == 1.0

    # Set pin 0 to 0.0
    logging.info("Setting pin %d to 0.0", test_pin)
    robot._io_status_controller_interface.set_io(fun=1, pin=test_pin, state=0.0)

    # Wait until the pin state has changed back
    end_time = time.time() + 5
    while pin_state and time.time() < end_time:
        rclpy.spin_once(robot.node, timeout_sec=0.1)
        if io_msg is not None:
            pin_state = io_msg.digital_out_states[test_pin].state

    assert pin_state == 0.0

    # Clean up io subscription
    robot.node.destroy_subscription(io_states_sub)


def test_trajectory(robot):
    """Test robot movement."""
    # Construct test trajectory
    test_trajectory = [
        (Duration(sec=6, nanosec=0), [0.0 for j in ROBOT_JOINTS]),
        (Duration(sec=9, nanosec=0), [-0.5 for j in ROBOT_JOINTS]),
        (Duration(sec=12, nanosec=0), [-1.0 for j in ROBOT_JOINTS]),
    ]

    trajectory = JointTrajectory(
        joint_names=[robot.tf_prefix + joint for joint in ROBOT_JOINTS],
        points=[
            JointTrajectoryPoint(positions=test_pos, time_from_start=test_time)
            for (test_time, test_pos) in test_trajectory
        ],
    )

    # Sending trajectory goal
    logging.info("Sending simple goal")
    goal_handle = robot._scaled_follow_joint_trajectory.send_goal(trajectory=trajectory)
    assert goal_handle.accepted

    # Verify execution
    result = robot._scaled_follow_joint_trajectory.get_result(
        goal_handle, TIMEOUT_EXECUTE_TRAJECTORY
    )
    assert result.error_code == FollowJointTrajectory.Result.SUCCESSFUL


def test_illegal_trajectory(robot):
    """
    Test trajectory server.

    This is more of a validation test that the testing suite does the right thing
    """
    # Construct test trajectory, the second point wrongly starts before the first
    test_trajectory = [
        (Duration(sec=6, nanosec=0), [0.0 for j in ROBOT_JOINTS]),
        (Duration(sec=3, nanosec=0), [-0.5 for j in ROBOT_JOINTS]),
    ]

    trajectory = JointTrajectory(
        joint_names=[robot.tf_prefix + joint for joint in ROBOT_JOINTS],
        points=[
            JointTrajectoryPoint(positions=test_pos, time_from_start=test_time)
            for (test_time, test_pos) in test_trajectory
        ],
    )

    # Send illegal goal
    logging.info("Sending illegal goal")
    goal_handle = robot._scaled_follow_joint_trajectory.send_goal(
        trajectory=trajectory,
    )

    # Verify the failure is correctly detected
    assert not goal_handle.accepted


def test_trajectory_scaled(robot):
    """Test robot movement."""
    # Construct test trajectory
    test_trajectory = [
        (Duration(sec=6, nanosec=0), [0.0 for j in ROBOT_JOINTS]),
        (Duration(sec=6, nanosec=500000000), [-1.0 for j in ROBOT_JOINTS]),
    ]

    trajectory = JointTrajectory(
        joint_names=[robot.tf_prefix + joint for joint in ROBOT_JOINTS],
        points=[
            JointTrajectoryPoint(positions=test_pos, time_from_start=test_time)
            for (test_time, test_pos) in test_trajectory
        ],
    )

    # Execute trajectory
    logging.info("Sending goal for robot to follow")
    goal_handle = robot._scaled_follow_joint_trajectory.send_goal(trajectory=trajectory)
    assert goal_handle.accepted

    # Verify execution
    result = robot._scaled_follow_joint_trajectory.get_result(
        goal_handle,
        TIMEOUT_EXECUTE_TRAJECTORY,
    )
    assert result.error_code == FollowJointTrajectory.Result.SUCCESSFUL


def test_trajectory_scaled_aborts_on_violation(robot):
    """Test that the robot correctly aborts the trajectory when the constraints are violated."""
    # Construct test trajectory
    if robot.mock_hardware:
        return True
    test_trajectory = [
        (Duration(sec=6, nanosec=0), [0.0 for j in ROBOT_JOINTS]),
        (
            Duration(sec=6, nanosec=50000000),
            [-1.0 for j in ROBOT_JOINTS],
        ),  # physically unfeasible
        (
            Duration(sec=8, nanosec=0),
            [-1.5 for j in ROBOT_JOINTS],
        ),  # physically unfeasible
    ]

    trajectory = JointTrajectory(
        joint_names=[robot.tf_prefix + joint for joint in ROBOT_JOINTS],
        points=[
            JointTrajectoryPoint(positions=test_pos, time_from_start=test_time)
            for (test_time, test_pos) in test_trajectory
        ],
    )

    last_joint_state = None

    def js_cb(msg):
        nonlocal last_joint_state
        last_joint_state = msg

    joint_state_sub = robot.node.create_subscription(JointState, "/joint_states", js_cb, 1)
    joint_state_sub  # prevent warning about unused variable

    # Send goal
    logging.info("Sending goal for robot to follow")
    goal_handle = robot._scaled_follow_joint_trajectory.send_goal(trajectory=trajectory)
    assert goal_handle.accepted

    # Get result
    result = robot._scaled_follow_joint_trajectory.get_result(
        goal_handle,
        TIMEOUT_EXECUTE_TRAJECTORY,
    )
    assert result.error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED

    state_when_aborted = last_joint_state

    # This section is to make sure the robot stopped moving once the trajectory was aborted
    time.sleep(2.0)
    # Ugly workaround since we want to wait for a joint state in the same thread
    while last_joint_state == state_when_aborted:
        rclpy.spin_once(robot.node)
    state_after_sleep = last_joint_state

    logging.info("Joint states before sleep:\t %s", state_when_aborted.position.tolist())
    logging.info("Joint states after sleep:\t %s", state_after_sleep.position.tolist())

    assert all(
        [abs(a - b) < 0.01 for a, b in zip(state_after_sleep.position, state_when_aborted.position)]
    )

    # # TODO: uncomment when JTC starts taking into account goal_time_tolerance from goal message
    # # see https://github.com/ros-controls/ros2_controllers/issues/249
    # # Now do the same again, but with a goal time constraint
    # # self.node.get_logger().info("Sending scaled goal with time restrictions")
    # #
    # # goal.goal_time_tolerance = Duration(nanosec=10000000)
    # # goal_response = self.call_action("/scaled_joint_trajectory_controller/follow_joint_trajectory", goal)
    # #
    # # self.assertEqual(goal_response.accepted, True)
    # #
    # # if goal_response.accepted:
    # #     result = self.get_result("/scaled_joint_trajectory_controller/follow_joint_trajectory", goal_response, TIMEOUT_EXECUTE_TRAJECTORY)
    # #     self.assertEqual(result.error_code, FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED)
    # #     self.node.get_logger().info("Received result GOAL_TOLERANCE_VIOLATED")


def test_passthrough_trajectory(robot):
    if robot.mock_hardware:
        return True
    assert robot._controller_manager_interface.switch_controller(
        strictness=SwitchController.Request.BEST_EFFORT,
        activate_controllers=["passthrough_trajectory_controller"],
        deactivate_controllers=["scaled_joint_trajectory_controller"],
    ).ok
    waypts = [
        [-1.58, -1.692, -1.4311, -0.0174, 1.5882, 0.0349],
        [-3, -1.692, -1.4311, -0.0174, 1.5882, 0.0349],
        [-1.58, -1.692, -1.4311, -0.0174, 1.5882, 0.0349],
    ]
    time_vec = [
        Duration(sec=4, nanosec=0),
        Duration(sec=8, nanosec=0),
        Duration(sec=12, nanosec=0),
    ]
    goal_tolerance = [
        JointTolerance(position=0.01, name=robot.tf_prefix + ROBOT_JOINTS[i])
        for i in range(len(ROBOT_JOINTS))
    ]
    goal_time_tolerance = Duration(sec=1, nanosec=0)
    test_trajectory = zip(time_vec, waypts)
    trajectory = JointTrajectory(
        points=[
            JointTrajectoryPoint(positions=pos, time_from_start=times)
            for (times, pos) in test_trajectory
        ],
        joint_names=[robot.tf_prefix + ROBOT_JOINTS[i] for i in range(len(ROBOT_JOINTS))],
    )
    goal_handle = robot._passthrough_forward_joint_trajectory.send_goal(
        trajectory=trajectory,
        goal_time_tolerance=goal_time_tolerance,
        goal_tolerance=goal_tolerance,
    )
    assert goal_handle.accepted
    if goal_handle.accepted:
        result = robot._passthrough_forward_joint_trajectory.get_result(
            goal_handle, TIMEOUT_EXECUTE_TRAJECTORY
        )
        assert result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
    # Test impossible goal tolerance, should fail.
    goal_tolerance = [
        JointTolerance(position=0.000000001, name=robot.tf_prefix + ROBOT_JOINTS[i])
        for i in range(len(ROBOT_JOINTS))
    ]
    goal_handle = robot._passthrough_forward_joint_trajectory.send_goal(
        trajectory=trajectory,
        goal_time_tolerance=goal_time_tolerance,
        goal_tolerance=goal_tolerance,
    )
    assert goal_handle.accepted
    if goal_handle.accepted:
        result = robot._passthrough_forward_joint_trajectory.get_result(
            goal_handle, TIMEOUT_EXECUTE_TRAJECTORY
        )
        assert result.error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED

    # Test impossible goal time
    goal_tolerance = [
        JointTolerance(position=0.01, name=robot.tf_prefix + ROBOT_JOINTS[i]) for i in range(6)
    ]
    goal_time_tolerance.sec = 0
    goal_time_tolerance.nanosec = 10
    goal_handle = robot._passthrough_forward_joint_trajectory.send_goal(
        trajectory=trajectory,
        goal_time_tolerance=goal_time_tolerance,
        goal_tolerance=goal_tolerance,
    )
    assert goal_handle.accepted
    if goal_handle.accepted:
        result = robot._passthrough_forward_joint_trajectory.get_result(
            goal_handle, TIMEOUT_EXECUTE_TRAJECTORY
        )
        assert result.error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
    assert robot._controller_manager_interface.switch_controller(
        strictness=SwitchController.Request.BEST_EFFORT,
        deactivate_controllers=["passthrough_trajectory_controller"],
        activate_controllers=["scaled_joint_trajectory_controller"],
    ).ok
