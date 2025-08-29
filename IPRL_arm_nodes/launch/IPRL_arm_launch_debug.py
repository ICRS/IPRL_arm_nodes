from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='IPRL_arm_nodes',
            executable='joy_2_command',
            name='joy_2_command',
            output='screen'
        ),
        Node(
            package='IPRL_arm_nodes',
            executable='serial_interface',
            name='arm_bridge',
            output='screen'
        ),
    ])
