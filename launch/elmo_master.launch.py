from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # CAN0 : Left
        Node(
            package='elmo_master',
            executable='elmo_master_exe',
            name='elmo_master_node_can0',
            namespace='can0',
            output='screen',
            parameters=[
                {'type': 'combined' },
                {'can_interface' : 'can0'},
                {'can_config_file' : 'config/elmo_can1.yaml'},
                {'resolution_position': 33554432}, # 2^25
                {'resolution_velocity': 65536}, # 2^25
                {'max_position': 3.14159}, # pi
                {'min_position': -3.14159}, # -pi
                {'max_velocity': 1.0}, # 1m/s (elmo), 5m/s (test motor) 
                {'wheel_radius': 0.08}, # 0.08m(elmo), 0.1m (test motor)
            ],
        ),
        # CAN1 : Right
        Node(
            package='elmo_master',
            executable='elmo_master_exe',
            name='elmo_master_node_can1',
            namespace='can1',
            output='screen',
            parameters=[
                {'type': 'combined'},
                {'can_interface' : 'can1'},
                {'can_config_file' : 'config/elmo_can1.yaml'},
                {'resolution_position': 33554432}, # 2^25 (elmo), 88 (test motor)
                {'resolution_velocity': 65536}, # 2^25 (elmo), 88 (test motor)
                {'max_position': 3.14159}, # pi
                {'min_position': -3.14159}, # -pi
                {'max_velocity': 1.0}, # 1m/s (elmo), 5m/s (test motor) 
                {'wheel_radius': 0.08}, # 0.08m(elmo), 0.1m (test motor)
            ]
        )
    ])