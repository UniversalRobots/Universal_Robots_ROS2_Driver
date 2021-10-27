import rclpy
from rclpy.action import ActionServer, server
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory


class FollowJointTrajectoryActionServer(Node):

    def __init__(self):
        super().__init__('follow_joint_trajectory_action_server')
        print("Starting UR Ignition action server")
        self.joint_trajectory_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            self.execute_callback)

    def execute_callback(self, goal_handle: server.ServerGoalHandle):
        self.get_logger().info('Executing goal...')
        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory # type: JointTrajectory
        self.joint_trajectory_publisher.publish(trajectory)
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    follow_joint_trajectory_action_server = FollowJointTrajectoryActionServer()
    rclpy.spin(follow_joint_trajectory_action_server)

if __name__ == '__main__':
    main()