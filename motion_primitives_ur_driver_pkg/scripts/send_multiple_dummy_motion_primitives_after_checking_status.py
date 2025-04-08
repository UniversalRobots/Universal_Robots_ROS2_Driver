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


class MotionPublisher(Node):
    def __init__(self):
        super().__init__('motion_publisher')

        # Publisher für MotionPrimitive-Nachrichten
        self.publisher_ = self.create_publisher(MotionPrimitive, '/motion_primitive_controller/reference', 10)

        # Subscriber für den State
        self.subscription = self.create_subscription(Int8, '/motion_primitive_controller/state', self.state_callback, 10)

        # Nachrichten vorbereiten
        self.messages = [msg1, msg2, msg3, msg4]
        # self.messages = [msg1, msg2]
        self.get_logger().info(f"Number of messages: {len(self.messages)}")

        self.current_msg_index = 0  # Start mit der ersten Nachricht

        self.print_error = True  # Flag für Fehlerausgabe --> only print once
        self.was_executing = False  # Flag für Ausführung --> nur einmal setzen

        # Erste Nachricht direkt senden
        self.send_next_message()

    def send_next_message(self):
        """ Sendet die nächste Nachricht in der Liste, falls vorhanden """
        self.was_executing = False
        self.get_logger().info(f"Sending message {self.current_msg_index + 1} of {len(self.messages)}")
        if self.current_msg_index < len(self.messages):
            msg = self.messages[self.current_msg_index]
            self.publisher_.publish(msg)
            self.get_logger().info(f'Sent message {self.current_msg_index + 1}: {msg}')
            self.current_msg_index += 1
        else:
            self.get_logger().info("All messages sent.")

    def state_callback(self, msg):
        """ Reagiert auf den aktuellen State """
        state = msg.data
        if state == 1:  # Ausführung
            if not self.was_executing:
                self.get_logger().info("Executing motion...")
                self.was_executing = True
        elif state == 2 and self.was_executing:  # Erfolg
            self.get_logger().info("Success: Motion completed!")
            self.send_next_message()
        elif state == 3 and self.print_error:  # Fehler
            self.print_error = False
            self.get_logger().error("Error: Motion failed!")


def main(args=None):
    rclpy.init(args=args)
    node = MotionPublisher()
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
