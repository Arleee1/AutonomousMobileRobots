from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_vel',
            default_value='/TTB10/cmd_vel',
            description='cmd_vel topic remap'
        ),
        Node(
            package='joy',
            namespace='/TTB10/',
            executable='ttb_joy',
            name='ttb_joy',
            output='screen',
            remappings=[('cmd_vel', LaunchConfiguration('cmd_vel'))],  # Remap cmd_vel
        )
    ])
