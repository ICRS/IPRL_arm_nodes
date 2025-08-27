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
        Node(
            package='joy',
            executable='joy_node',
            name='arm_joy',
            remappings=[("/joy","/arm/joy")],
        ),
        Node(
            package='IPRL_arm_nodes',
            executable='visualiser',
            name='arm_visualiser',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='arm_rviz',
            output='screen'
        )
    ])
