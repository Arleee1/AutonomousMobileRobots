from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    wander_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('wander'), '/launch/wander.launch.py']),
        launch_arguments={'cmd_vel': '/TTB10/wander/cmd_vel'}.items(),  # remap cmd_vel topic
    )

    joy_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('joy'), '/launch/ttb_joy.launch.py']),
        launch_arguments={'cmd_vel': '/TTB10/joy/cmd_vel'}.items(),  # remap cmd_vel topic
    )

    cc_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('cc_variable'), '/launch/ttb_pid.launch.py']),
        launch_arguments={'cmd_vel': '/TTB10/cc/cmd_vel'}.items(),  # remap cmd_vel topic
    )

    return LaunchDescription([
        wander_launch_file,
        joy_launch_file,
        cc_launch_file,
        Node(
            package='state_machine',
            namespace='/TTB10/',
            executable='state_machine_node',
            name='state_machine_node',
            output='screen',
        )
    ])
