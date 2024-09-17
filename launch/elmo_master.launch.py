from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # # CAN0
        # Node(
        #     package='elmo_master',
        #     executable='elmo_master_exe',
        #     name='elmo_master_node_can0',
        #     output='screen',
        #     parameters=[
        #         {'type': 'combined' },
        #         {'can_interface' : 'can0'},
        #         {'can_config_file' : 'config/elmo_can0.yaml'}
        #     ]
        # ),
        # CAN1
        Node(
            package='elmo_master',
            executable='elmo_master_exe',
            name='elmo_master_node_can1',
            output='screen',
            parameters=[
                {'type': 'combined'},
                {'can_interface' : 'can1'},
                {'can_config_file' : 'config/elmo_can1.yaml'}
            ]
        )
    ])