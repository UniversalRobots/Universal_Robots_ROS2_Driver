import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()
    description_package_path = get_package_share_directory('ur_description')
    robot_description_file = os.path.join(description_package_path, 'urdf', 'ur5.urdf.xacro')

    controller_package_path = get_package_share_directory('ur_robot_driver')
    robot_controller_file = os.path.join(controller_package_path, 'config', 'ur5_controllers.yaml')
    
    with open(robot_description_file, 'r') as infile:
        descr = infile.read()
    robot_description = {'robot_description': descr}


    return LaunchDescription([
      Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controller_file],
        output={
          'stdout': 'screen',
          'stderr': 'screen',
          },
        )
    ])