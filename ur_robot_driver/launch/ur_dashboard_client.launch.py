from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ur_robot_driver",
            executable="dashboard_client",
            name="dashboard_client",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"robot_ip": "10.0.1.186"}
            ]
        )
    ])
