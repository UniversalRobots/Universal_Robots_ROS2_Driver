#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers


def main(args=None):
    """
    Waits for the controller manager service to start and respond successfully.
    This ensures the robot controllers only start when the system is fully ready.
    """

    rclpy.init(args=args)
    node = Node("ur_controller_manager_awaiter")

    use_mock_hardware = node.declare_parameter("use_mock_hardware", False).value
    check_interval = node.declare_parameter("check_interval", 10.0).value
    timeout = node.declare_parameter("service_response_timeout", 5.0).value

    if use_mock_hardware:
        node.get_logger().info("Mock hardware detected. Skipping service validation.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    node.get_logger().info("Awaiting controller_manager service availability...")

    while rclpy.ok():
        if not client.wait_for_service(timeout_sec=check_interval):
            node.get_logger().info("Service is not available yet. Retrying...")
            continue

        node.get_logger().info("Service found in registry. Pinging...")
        future = client.call_async(ListControllers.Request())

        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)

        if future.done():
            try:
                future.result()
                node.get_logger().info("Service responded successfully! Unblocking spawners...")
                break
            except Exception as e:
                node.get_logger().warning(f"Service call failed: {e}. Retrying...")
        else:
            node.get_logger().warning(f"Timeout: Service did not respond within {timeout}s. Canceling and retrying...")
            future.cancel()

        time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
