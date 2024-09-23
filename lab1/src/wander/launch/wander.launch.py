from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wander',
            namespace='/TTB10/',
            executable='wander_node',
            name='wander_node',
            output='screen',
            parameters=[
                {'const_speed': 0.3}
                # {'const_speed': '0.0'} # for state machine
            ],
            remappings=[
                # ('/TTB10/cmd_vel', '/TTB10/wander/cmd_vel') # remap for state machine
            ]
        ),
        Node(
            package='wander',
            namespace='/TTB10/front',
            executable='ir_distance_node',
            name='ir_distance_node',
            output='screen',
            parameters=[
                {'ir_index': 3}
            ],
            remappings=[
                ('/TTB10/front/ir_range', '/TTB10/ir_range'),  # Remap the topic to /TTB10/ir_range
                ('/TTB10/front/ir_intensity', '/TTB10/ir_intensity')  # Remap the topic to /TTB10/ir_intensity
            ]
        ),
        Node(
            package='wander',
            namespace='/TTB10/left',
            executable='ir_distance_node',
            name='ir_distance_node',
            output='screen',
            parameters=[
                {'ir_index': 2}
            ],
            remappings=[
                ('/TTB10/left/ir_range', '/TTB10/ir_range'),  # Remap the topic to /TTB10/ir_range
                ('/TTB10/left/ir_intensity', '/TTB10/ir_intensity')  # Remap the topic to /TTB10/ir_intensity
            ]
        ),
        Node(
            package='wander',
            namespace='/TTB10/right',
            executable='ir_distance_node',
            name='ir_distance_node',
            output='screen',
            parameters=[
                {'ir_index': 4}
            ],
            remappings=[
                ('/TTB10/right/ir_range', '/TTB10/ir_range'),  # Remap the topic to /TTB10/ir_range
                ('/TTB10/right/ir_intensity', '/TTB10/ir_intensity')  # Remap the topic to /TTB10/ir_intensity
            ]
        )
    ])
