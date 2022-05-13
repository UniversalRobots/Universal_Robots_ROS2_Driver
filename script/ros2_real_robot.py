import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from math import pi

class Trajectory(Node):
    ## Init program
    def __init__(self):
        ## Init ROS node

        '''insira um nome para o nó no "node_name"'''
        super().__init__("node_name")

        ## Init joints configuration callback
        self.joints_position = JointState( )
        self.joints_names = ['shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        ## Define publishers
        self.publisher = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)       

        ## Define subscribers
        self.subscriber = self.create_subscription(JointState, '/joint_states', self.callback, 10)

        ## Define timer
        self.timer_fk = self.create_timer(1.0, self.forward_kinematics)
        
    # Joints angles callback
    def callback(self, data):
        self.joints_position = data.position

    def forward_kinematics(self):
        ## FK Points
        fk = JointTrajectoryPoint( )

        '''Aqui insere os angulos de junta para o robo mexer'''
        fk.positions = [0.0, -pi/2, 0.0, -pi/2, 0.0, 0.0]
        fk.time_from_start = Duration(sec=8)

        ## Go to Points
        go_to = JointTrajectory( )
        go_to.joint_names = self.joints_names
        go_to.points = [fk]

        self.get_logger().info('Publish')
        self.publisher.publish(go_to)
        
    
if __name__ == '__main__':
    rclpy.init( )
    # pdb.set_trace( )
    try:
        traj = Trajectory( )
        traj.forward_kinematics( )

        rclpy.spin(traj)
        traj.destroy_node()
        rclpy.shutdown()
    except:
        pass
