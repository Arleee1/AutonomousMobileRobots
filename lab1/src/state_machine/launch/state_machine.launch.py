from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # another_launch_file = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([get_package_share_directory('another_package'), '/launch/another_launch.py']),
    #     launch_arguments={'some_arg': 'some_value'}.items(),
    # )

    return LaunchDescription([
        # another_launch_file,
        Node(
            package='my_package',
            executable='my_node',
            name='my_node_name'
        )
    ])
