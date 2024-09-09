from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Throttle node
        Node(
            package='elmo_master',
            executable='elmo_master_exe',
            name='elmo_master_throttle_node',
            output='screen',
            parameters=[
                {'type': 'throttle'}
            ]
        ),
        # Steering node
        Node(
            package='elmo_master',
            executable='elmo_master_exe',
            name='elmo_master_steering_node',
            output='screen',
            parameters=[
                {'type': 'steering'}
            ]
        )
    ])