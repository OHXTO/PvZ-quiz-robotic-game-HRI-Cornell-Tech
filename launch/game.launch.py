from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # Color detector node
        Node(
            package='lab5',
            executable='color_detector_node',
            name='color_detector',
            output='screen'
        ),

        # Game controller node
        Node(
            package='lab5',
            executable='game_controller',
            name='game_controller',
            output='screen'
        )
    ])

