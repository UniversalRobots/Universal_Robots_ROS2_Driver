#!/usr/bin/env python3
import sys
import socket
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers

class UrHardwareAwaiter(Node):
    """
    Blocks controller deployment in the launcher until ALL UR robot control interfaces 
    are fully initialized AND the ROS 2 controller_manager service actually responds.
    """
    def __init__(self):
        """
        Initializes the node, declares parameters, sets up the ROS 2 service client,
        configures the periodic timer, and triggers the initial instant hardware check.
        """
        
        super().__init__('ur_hardware_awaiter')
        
        self.declare_parameter('robot_ip', '192.168.56.101')
        self.declare_parameter('check_interval', 10.0) 
        self.declare_parameter('connection_timeout', 1.0)
        self.declare_parameter('service_response_timeout', 5.0)

        self.robot_ip = self.get_parameter('robot_ip').value
        self.check_interval = self.get_parameter('check_interval').value
        self.connection_timeout = self.get_parameter('connection_timeout').value
        self.service_response_timeout = self.get_parameter('service_response_timeout').value
        self.service_call_future = None
        self.service_call_deadline = None

        self.ur_ports = [30001, 30002, 30004]
        self.ur_ready = False
        self.waiting_for_service_response = False

        self.client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        
        self.get_logger().info(f"Awaiting robot initialization at IP {self.robot_ip}...")
        
        self.timer = self.create_timer(self.check_interval, self.check_status_callback)

        self.check_status_callback()

    def check_tcp_connection(self):
        """
        Attempts to open TCP connections to all required UR control ports to verify 
        that the physical robot or simulator network interfaces are fully reachable.
        """

        for port in self.ur_ports:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(self.connection_timeout)
                    s.connect((self.robot_ip, port))
            except (socket.timeout, ConnectionRefusedError, OSError):
                return False
        return True

    def check_status_callback(self):
        """
        Periodic execution step that sequentially verifies hardware sockets and ROS 2 service.
        """

        if self.waiting_for_service_response:
            if self.service_call_future is not None and self.service_call_future.done():
                return

            if self.service_call_deadline is not None and self.get_clock().now() > self.service_call_deadline:
                self.get_logger().warning(
                    "Service call timeout after %.1f seconds. Resetting and retrying...",
                    self.service_response_timeout,
                )
                if self.service_call_future is not None:
                    self.service_call_future.cancel()
                self.waiting_for_service_response = False
                self.service_call_future = None
                self.service_call_deadline = None
            return

        if not self.check_tcp_connection():
            self.get_logger().info(f"System is still initializing. Retrying in {self.check_interval} seconds...")
            return

        if not self.client.wait_for_service(timeout_sec=0.1):
            self.get_logger().info("Sockets ready, but ROS 2 controller_manager service is not up yet...")
            return

        self.get_logger().info("Service found in registry. Pinging to verify it is responsive...")
        self.waiting_for_service_response = True

        req = ListControllers.Request()
        self.service_call_future = self.client.call_async(req)
        self.service_call_deadline = self.get_clock().now() + Duration(seconds=self.service_response_timeout)
        self.service_call_future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, call):
        """
        Processes the outcome of the asynchronous service call, evaluating success 
        to either unblock the lifecycle launcher or reset guards for future retries.
        """

        try:
            call.result()
            self.get_logger().info("Service responded successfully. Controller spawner is unblocked.")
            self.ur_ready = True
            self.timer.cancel()
            if rclpy.ok():
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().warning(f"Service call failed: {e}. Retrying...")
            self.waiting_for_service_response = False
            self.service_call_future = None
            self.service_call_deadline = None


def main(args=None):
    rclpy.init(args=args)
    node = UrHardwareAwaiter()

    rclpy.spin(node)

    exit_code = 0 if node.ur_ready else 1
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()