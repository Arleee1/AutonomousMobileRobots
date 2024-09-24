from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            namespace='/TTB10/',
            executable='ttb_joy',
            name='ttb_joy',
            output='screen',
        )
    ])
