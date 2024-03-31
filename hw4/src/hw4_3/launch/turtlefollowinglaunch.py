from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='hw4_3',
            executable='spawner',
            name='turtle_spawn',
            parameters=[
                {}
            ]            
        ),
        Node(
            package='hw4_3',
            executable='controller',
            name='turtle_control',
            parameters=[
                {'avoidwalls': 0}
            ]            
        )
    ])