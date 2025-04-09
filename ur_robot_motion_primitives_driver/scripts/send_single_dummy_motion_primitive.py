#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from geometry_msgs.msg import PoseStamped
from industrial_robot_motion_interfaces.msg import MotionPrimitive, MotionArgument

class DummyPublisher(Node):
    def __init__(self):
        super().__init__('dummy_publisher')
        
        # Publisher for the MotionPrimitive message
        self.publisher_ = self.create_publisher(MotionPrimitive, '/motion_primitive_controller/reference', 10)
        
        # Timer to call the publishing function every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = MotionPrimitive()
    
        msg.type = 50
        # msg.blend_radius = 0.051
        
        # Add additional arguments 
        arg_vel = MotionArgument()
        arg_vel.argument_name = "velocity"
        arg_vel.argument_value = 1.03

        arg_acc = MotionArgument()
        arg_acc.argument_name = "acceleration"
        arg_acc.argument_value = 0.57
        # msg.additional_arguments = [arg_vel, arg_acc]

        arg_time = MotionArgument()
        arg_time.argument_name = "move_time"
        arg_time.argument_value = 0.0
        msg.additional_arguments = [arg_vel, arg_acc, arg_time]
        
        # Create a PoseStamped message (Cartesian position)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = -0.203
        pose.pose.position.y = 0.263
        pose.pose.position.z = 0.559
        pose.pose.orientation.x = -0.2734
        pose.pose.orientation.y = -0.4930
        pose.pose.orientation.z = -0.6086
        pose.pose.orientation.w = 0.5584
        msg.poses = [pose]  # Add the pose to the message
        # { -0.203, 0.263, 0.559, 0.68, -1.083, -2.076 }
        # Euler_XYZ(0.68, -1.083, -2.076) = Quaternion_XYZW(0.006, -0.009, -0.018, 1.0) (https://quaternions.online/)
        # Euler_XYZ(0.68, -1.083, -2.076) = Quaternion_XYZW(0.5637225, -0.0006527, -0.7832093, 0.2622969) (https://www.andre-gaschler.com/rotationconverter/)
        # Euler_XYZ(0.68, -1.083, -2.076) = Quaternion_XYZW(−0.2734,−0.4930,−0.6086,0.5584) (ChatGPT)
        
        # Set joint positions
        # msg.joint_positions = [0.51, 0.52, 0.53, 0.54, 0.55, 0.56]
        
        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')

def main(args=None):
    rclpy.init(args=args)
    dummy_publisher = DummyPublisher()
    
    # rclpy.spin(dummy_publisher) # Spin to keep the node active and publishing messages
    rclpy.spin_once(dummy_publisher)  # Only spin once to call publish_message()

    # Clean up at the end
    dummy_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
