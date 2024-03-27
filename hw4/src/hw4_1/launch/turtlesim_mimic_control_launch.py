from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim1',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim1/turtle1/cmd_vel'),
            ]
        ),  
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim2',
            remappings=[
                ('/input/pose', '/turtlesim2/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        ),   
        Node(
            package='turtlesim',
            namespace='turtlesim3',
            executable='turtlesim_node',
            name='sim3',
            remappings=[
                ('/input/pose', '/turtlesim3/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim3/turtle1/cmd_vel'),
            ]
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim4',
            executable='turtlesim_node',
            name='sim4',
            remappings=[
                ('/input/pose', '/turtlesim4/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim4/turtle1/cmd_vel'),
            ]
        ),      
    ])