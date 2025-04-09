#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from industrial_robot_motion_interfaces.msg import MotionPrimitive, MotionArgument

# 4 Motion primitives aus: https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_client_library/doc/examples/instruction_executor.html#instruction-executor-example
msg1 = MotionPrimitive()
msg1.type = MotionPrimitive.LINEAR_JOINT
msg1.joint_positions = [ -1.57, -1.57, 0, 0, 0, 0]
# msg1.blend_radius = 0.5
msg1.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=1.0),
    MotionArgument(argument_name="acceleration", argument_value=1.0)]

msg2 = MotionPrimitive()
msg2.type = MotionPrimitive.LINEAR_JOINT
# msg2.blend_radius = 0.5
msg2.joint_positions = [-1.57, -1.6, 1.6, -0.7, 0.7, 0.2]
msg2.additional_arguments = [
    MotionArgument(argument_name="move_time", argument_value=2.0)]

msg3 = MotionPrimitive()
msg3.type = MotionPrimitive.LINEAR_CARTESIAN
msg3.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=2.0),
    MotionArgument(argument_name="acceleration", argument_value=0.5),
    MotionArgument(argument_name="move_time", argument_value=0.0)]
pose3 = PoseStamped()
pose3.pose.position.x = -0.203
pose3.pose.position.y = 0.263
pose3.pose.position.z = 0.559
pose3.pose.orientation.x = -0.2734
pose3.pose.orientation.y = -0.4930
pose3.pose.orientation.z = -0.6086
pose3.pose.orientation.w = 0.5584
msg3.poses = [pose3]

msg4 = MotionPrimitive()
msg4.type = MotionPrimitive.LINEAR_CARTESIAN
msg4.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=1.0),
    MotionArgument(argument_name="acceleration", argument_value=0.5),
    MotionArgument(argument_name="move_time", argument_value=2.0)]
pose4 = PoseStamped()
pose4.pose.position.x = -0.203
pose4.pose.position.y = 0.463
pose4.pose.position.z = 0.559
pose4.pose.orientation.x = -0.2734
pose4.pose.orientation.y = -0.4930
pose4.pose.orientation.z = -0.6086
pose4.pose.orientation.w = 0.5584
msg4.poses = [pose4]

# CIRC Befehl nicht unterstützt --> triggert motion sequence
msg5 = MotionPrimitive()
msg5.type = MotionPrimitive.CIRCULAR_CARTESIAN
msg5.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.5),
    MotionArgument(argument_name="acceleration", argument_value=0.1)]
# urcl::Pose(-0.150, 0.350, 0.550, 0.68, -1.0, -2.0),  // via (Zwischenpose)
# urcl::Pose(-0.100, 0.400, 0.550, 0.68, -1.0, -2.0),  // target (Zielpose)
pose5_via = PoseStamped()
pose5_via.pose.position.x = -0.150
pose5_via.pose.position.y = 0.350
pose5_via.pose.position.z = 0.550
pose5_via.pose.orientation.x = -0.2222
pose5_via.pose.orientation.y = -0.4905
pose5_via.pose.orientation.z = -0.6098
pose5_via.pose.orientation.w = 0.58155
pose5_goal = PoseStamped()
pose5_goal.pose.position.x = -0.100
pose5_goal.pose.position.y = 0.400
pose5_goal.pose.position.z = 0.550
pose5_goal.pose.orientation.x = -0.2222
pose5_goal.pose.orientation.y = -0.490
pose5_goal.pose.orientation.z = -0.608
pose5_goal.pose.orientation.w = 0.58155
msg5.poses = [pose5_goal, pose5_via]    # fist pose is goal, second is via point

# Dummy msg to execute motion sequence
msg6 = MotionPrimitive()
msg6.type = 33  # Dummy type
msg6.joint_positions = [ -1.57, -1.57, 0, 0, 0, 0]
msg6.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=1.0),
    MotionArgument(argument_name="acceleration", argument_value=1.0)]

msg_start_sequence = MotionPrimitive()
msg_start_sequence.type = MotionPrimitive.MOTION_SEQUENCE_START

msg_end_sequence = MotionPrimitive()
msg_end_sequence.type = MotionPrimitive.MOTION_SEQUENCE_END

class MotionPublisher(Node):
    def __init__(self):
        super().__init__('motion_publisher')

        # Publisher für MotionPrimitive-Nachrichten
        self.publisher_ = self.create_publisher(MotionPrimitive, '/motion_primitive_controller/reference', 10)

        # Nachrichten vorbereiten
        # self.messages = [msg_start_sequence, msg1, msg2, msg3, msg4, msg5, msg_end_sequence]
        self.messages = [msg1, msg2, msg3, msg4, msg5, msg_start_sequence, msg1, msg2, msg3, msg4, msg5, msg_end_sequence, msg_start_sequence, msg1, msg2, msg_end_sequence, msg1, msg2]
        # self.messages = [msg1, msg2, msg3, msg4, msg5, msg6]
        # self.messages = [msg1, msg2, msg3, msg4, msg1, msg2, msg3, msg4]
        self.get_logger().info(f"Number of messages: {len(self.messages)}")

        # Alle Nachrichten direkt senden
        self.send_all_messages()
    

    def send_all_messages(self):
        """ Sendet alle Nachrichten direkt nacheinander """
        for i, msg in enumerate(self.messages):
            self.get_logger().info(f"Sending message {i + 1} of {len(self.messages)}")
            self.publisher_.publish(msg)
            self.get_logger().info(f"Sent message {i + 1}: {msg}")


def main(args=None):
    rclpy.init(args=args)
    node = MotionPublisher()
    rclpy.spin_once(node, timeout_sec=1)  # Kurzes Warten, um Nachrichten zu senden

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
