from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='stepper_control',  
        #     executable='stepper_node',
        #     name='stepper_control',
        #     parameters=[
        #         {"done_signal_topic": "/stepper/done"} 
        #     ]
        # ),
        # Node(
        #     package='integration',  
        #     executable='integration_node',  
        #     name='integration',
        #     parameters=[
        #         {"start_signal_topic": "/stepper/done"}  
        #     ]
        # ),
        Node(
            package='camera_node',  
            executable='camera_node',  
            name='camera',
            parameters=[
                {"start_signal_topic": "ArucoID"}  
            ]
        ),
        # Node(
        #     package='bvm_node',  
        #     executable='bvm_node',  
        #     name='bvm',
        #     parameters=[
        #         {"done_signal_topic": "ArucoID"}  
        #     ]
        # ),
    ])