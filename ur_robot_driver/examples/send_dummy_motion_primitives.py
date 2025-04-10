#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from industrial_robot_motion_interfaces.msg import MotionPrimitive, MotionArgument

# Motion primitives from: https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_client_library/doc/examples/instruction_executor.html#instruction-executor-example
msg_moveJ_1 = MotionPrimitive()
msg_moveJ_1.type = MotionPrimitive.LINEAR_JOINT
msg_moveJ_1.joint_positions = [ -1.57, -1.57, 0, 0, 0, 0]
msg_moveJ_1.blend_radius = 0.1
msg_moveJ_1.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=1.0),
    MotionArgument(argument_name="acceleration", argument_value=1.0)]

msg_moveJ_2 = MotionPrimitive()
msg_moveJ_2.type = MotionPrimitive.LINEAR_JOINT
msg_moveJ_2.blend_radius = 0.1
msg_moveJ_2.joint_positions = [-1.57, -1.6, 1.6, -0.7, 0.7, 0.2]
msg_moveJ_2.additional_arguments = [
    MotionArgument(argument_name="move_time", argument_value=2.0)]

msg_moveL_1 = MotionPrimitive()
msg_moveL_1.type = MotionPrimitive.LINEAR_CARTESIAN
msg_moveL_1.blend_radius = 0.05
msg_moveL_1.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=2.0),
    MotionArgument(argument_name="acceleration", argument_value=0.5),
    MotionArgument(argument_name="move_time", argument_value=0.0)]
pose_L1 = PoseStamped()
pose_L1.pose.position.x = -0.203
pose_L1.pose.position.y = 0.263
pose_L1.pose.position.z = 0.559
pose_L1.pose.orientation.x = -0.2734
pose_L1.pose.orientation.y = -0.4930
pose_L1.pose.orientation.z = -0.6086
pose_L1.pose.orientation.w = 0.5584
msg_moveL_1.poses = [pose_L1]

msg_moveL_2 = MotionPrimitive()
msg_moveL_2.type = MotionPrimitive.LINEAR_CARTESIAN
msg_moveL_2.blend_radius = 0.05
msg_moveL_2.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=1.0),
    MotionArgument(argument_name="acceleration", argument_value=0.5),
    MotionArgument(argument_name="move_time", argument_value=2.0)]
pose_L2 = PoseStamped()
pose_L2.pose.position.x = -0.203
pose_L2.pose.position.y = 0.463
pose_L2.pose.position.z = 0.559
pose_L2.pose.orientation.x = -0.2734
pose_L2.pose.orientation.y = -0.4930
pose_L2.pose.orientation.z = -0.6086
pose_L2.pose.orientation.w = 0.5584
msg_moveL_2.poses = [pose_L2]

msg_moveC_1 = MotionPrimitive()
msg_moveC_1.type = MotionPrimitive.CIRCULAR_CARTESIAN
msg_moveC_1.blend_radius = 0.0
msg_moveC_1.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.5),
    MotionArgument(argument_name="acceleration", argument_value=0.1)]
# urcl::Pose(-0.150, 0.350, 0.550, 0.68, -1.0, -2.0),  // via_pose
# urcl::Pose(-0.100, 0.400, 0.550, 0.68, -1.0, -2.0),  // target_pose
pose_C1_via = PoseStamped()
pose_C1_via.pose.position.x = -0.150
pose_C1_via.pose.position.y = 0.350
pose_C1_via.pose.position.z = 0.550
pose_C1_via.pose.orientation.x = -0.2222
pose_C1_via.pose.orientation.y = -0.4905
pose_C1_via.pose.orientation.z = -0.6098
pose_C1_via.pose.orientation.w = 0.58155
pose_C1_goal = PoseStamped()
pose_C1_goal.pose.position.x = -0.100
pose_C1_goal.pose.position.y = 0.400
pose_C1_goal.pose.position.z = 0.550
pose_C1_goal.pose.orientation.x = -0.2222
pose_C1_goal.pose.orientation.y = -0.490
pose_C1_goal.pose.orientation.z = -0.608
pose_C1_goal.pose.orientation.w = 0.58155
msg_moveC_1.poses = [pose_C1_goal, pose_C1_via]    # first pose is goal, second is via point

msg_start_sequence = MotionPrimitive()
msg_start_sequence.type = MotionPrimitive.MOTION_SEQUENCE_START

msg_end_sequence = MotionPrimitive()
msg_end_sequence.type = MotionPrimitive.MOTION_SEQUENCE_END

class MotionPublisher(Node):
    def __init__(self):
        super().__init__('motion_publisher')

        self.publisher_ = self.create_publisher(MotionPrimitive, '/motion_primitive_controller/reference', 10)

        self.messages = [msg_moveJ_1, msg_moveJ_2, msg_moveL_1, msg_moveL_2, msg_moveC_1, msg_start_sequence, msg_moveJ_1, msg_moveJ_2, msg_moveL_1, msg_moveL_2, msg_moveC_1, msg_end_sequence, msg_start_sequence, msg_moveJ_1, msg_moveJ_2, msg_end_sequence, msg_moveJ_1, msg_moveJ_2]
        self.get_logger().info(f"Number of messages: {len(self.messages)}")

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
    rclpy.spin_once(node, timeout_sec=1) 

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
