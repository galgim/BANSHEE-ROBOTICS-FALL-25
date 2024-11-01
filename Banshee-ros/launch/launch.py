from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stepper_package',  
            executable='stepper_node',
            name='stepper_control',
            parameters=[
                {"done_signal_topic": "/stepper/done"} 
            ]
        ),
        Node(
            package='integration_package',  
            executable='integration_node',  
            name='integration',
            parameters=[
                {"start_signal_topic": "/stepper/done"}  
            ]
        ),

    ])