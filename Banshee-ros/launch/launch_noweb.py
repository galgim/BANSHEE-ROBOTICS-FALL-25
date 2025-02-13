from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stepper_control',  
            executable='stepper_node',
            name='stepper_control',
        ),
        Node(
            package='integration',  
            executable='integration_node',  
            name='integration',
        ),
        Node(
            package='camera_node',
            executable='camera_no_web',
            name='camera_node',
        ),
    ])