"""Joystick teleop for my_arm (ROS 2 Jazzy, moveit_servo 2.12).

Left stick fwd/back   -> arm forward/back  (arm_teleop 2-link IK -> Servo joint jog)
Right stick up/down   -> arm up/down       (arm_teleop 2-link IK -> Servo joint jog)
Left stick left/right -> base left/right   (evdev_joy /base_cmd -> hardware plugin)
Nothing moves unless the deadman button is held.

  ros2 launch my_arm_description demo_joystick.launch.py                         # real arm
  ros2 launch my_arm_description demo_joystick.launch.py use_mock_hardware:=true # no hardware
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder

DEFAULT_SERIAL_PORT = (
    '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
)


def generate_launch_description():
    # ==========================================
    # 1. SETUP & CONFIG
    # ==========================================
    description_pkg = 'my_arm_description'
    moveit_pkg      = 'my_arm_moveit_config'

    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    serial_port = LaunchConfiguration('serial_port')

    moveit_config = (
        MoveItConfigsBuilder("my_arm", package_name=moveit_pkg)
        .robot_description(mappings={
            "use_mock_hardware": use_mock_hardware,
            "serial_port": serial_port,
        })
        .to_moveit_configs()
    )

    # Servo parameters must live under the 'moveit_servo' namespace in Jazzy.
    servo_params = {
        'moveit_servo': ParameterBuilder(moveit_pkg).yaml('config/servo_config.yaml').to_dict()
    }

    rviz_config_path = PathJoinSubstitution([FindPackageShare(moveit_pkg), "config", "moveit.rviz"])
    ros2_controllers_path = PathJoinSubstitution([FindPackageShare(moveit_pkg), "config", "ros2_controllers.yaml"])

    # ==========================================
    # 2. DEFINE NODES
    # ==========================================

    # A. Custom Evdev Driver (reads the joystick; publishes /joy and /base_cmd)
    joy_driver_node = Node(
        package=description_pkg,
        executable='evdev_joy',
        output='screen'
    )

    # B. Arm teleop (Joy -> shoulder/elbow speeds via 2-link IK, only while the deadman is held)
    arm_teleop_node = Node(
        package=description_pkg,
        executable='arm_teleop',
        output='screen'
    )

    # C. MoveIt Servo (joint jog -> joint trajectory for arm_controller, with limits/smoothing)
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output='screen',
    )

    # D. RViz (Visualization)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ]
    )

    # E. Robot State Publisher (publishes TF and /robot_description)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description]
    )

    # F. Controller Manager (reads the robot description from the topic, as in Jazzy)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ros2_controllers_path],
        remappings=[('/controller_manager/robot_description', '/robot_description')],
        output='screen'
    )

    # ==========================================
    # 3. CONTROLLER SPAWNERS
    # ==========================================
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    # Once the controllers are up: start RViz and make sure Servo is in JOINT_JOG mode (0)
    # (there is no start_servo service in Jazzy).
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    servo_joint_jog_mode = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/servo_node/switch_command_type',
             'moveit_msgs/srv/ServoCommandType', '{command_type: 0}'],
        output='screen'
    )
    delayed_servo_joint_jog_mode = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[servo_joint_jog_mode],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock ros2_control hardware instead of the ESP32 serial hardware.'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value=DEFAULT_SERIAL_PORT,
            description='ESP32 serial port.'
        ),
        joy_driver_node,
        arm_teleop_node,
        servo_node,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        delay_rviz,
        delayed_servo_joint_jog_mode,
    ])
