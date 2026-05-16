from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    port_arg = DeclareLaunchArgument("port", default_value="/dev/ttyUSB0")
    baud_arg = DeclareLaunchArgument("baud", default_value="115200")
    read_timeout_arg = DeclareLaunchArgument("read_timeout_ms", default_value="20")
    write_timeout_arg = DeclareLaunchArgument("write_timeout_ms", default_value="200")
    flush_arg = DeclareLaunchArgument("flush_on_write", default_value="false")
    use_console_arg = DeclareLaunchArgument("use_console", default_value="false")

    bridge_node = Node(
        package="my_arm_serial_bridge",
        executable="serial_bridge",
        name="raw_serial_bridge",
        output="screen",
        parameters=[
            {"port": LaunchConfiguration("port")},
            {"baud": LaunchConfiguration("baud")},
            {"read_timeout_ms": LaunchConfiguration("read_timeout_ms")},
            {"write_timeout_ms": LaunchConfiguration("write_timeout_ms")},
            {"flush_on_write": LaunchConfiguration("flush_on_write")},
        ],
    )

    console_node = Node(
        package="my_arm_serial_bridge",
        executable="serial_console",
        name="raw_serial_console",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_console")),
    )

    return LaunchDescription(
        [
            port_arg,
            baud_arg,
            read_timeout_arg,
            write_timeout_arg,
            flush_arg,
            use_console_arg,
            bridge_node,
            console_node,
        ]
    )
