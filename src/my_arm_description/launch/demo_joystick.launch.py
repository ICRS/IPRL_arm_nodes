from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
import yaml
from ament_index_python.packages import get_package_share_directory

# --- HELPER FUNCTION TO LOAD YAML ---
def load_yaml(package_name, file_path):
    try:
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
        if not os.path.exists(absolute_file_path):
            print(f"[ERROR] File not found: {absolute_file_path}")
            return None
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"[ERROR] Could not load yaml {file_path}: {e}")
        return None

def generate_launch_description():
    # ==========================================
    # 1. SETUP & CONFIG
    # ==========================================
    description_pkg = 'my_arm_description'
    moveit_pkg      = 'my_arm_moveit_config'

    # Load Servo Config (Ensuring we use the name you confirmed: servo_config.yaml)
    servo_yaml = load_yaml(moveit_pkg, 'config/servo_config.yaml')
    kin_yaml = load_yaml(moveit_pkg, 'config/kinematics.yaml')
    
    # Safety exit if file is missing
    if servo_yaml is None or kin_yaml is None:
        return LaunchDescription([])
    
    robot_description_kinematics = {'robot_description_kinematics': kin_yaml}
    servo_params = {
        'moveit_servo': servo_yaml,
        'move_group_name': 'arm',
    }
    # ==========================================
    # 2. ROBOT DESCRIPTION (URDF & SRDF)
    # ==========================================
    # We use ParameterValue(..., value_type=str) to fix the YAML parsing error
    
    # URDF from MoveIt config package so ros2_control initial positions are applied.
    initial_positions_path = PathJoinSubstitution([FindPackageShare(moveit_pkg), "config", "initial_positions.yaml"])
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([FindPackageShare(moveit_pkg), "config", "my_arm.urdf.xacro"]), " ",
            "initial_positions_file:=", initial_positions_path,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # SRDF
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="cat")]), " ",
            PathJoinSubstitution([FindPackageShare(moveit_pkg), "config", "my_arm.srdf"]),
        ]
    )
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    # ==========================================
    # 3. FILE PATHS
    # ==========================================
    joy_config_path = PathJoinSubstitution([FindPackageShare(moveit_pkg), "config", "xbox_mapping.yaml"])
    rviz_config_path = PathJoinSubstitution([FindPackageShare(moveit_pkg), "config", "moveit.rviz"])
    ros2_controllers_path = PathJoinSubstitution([FindPackageShare(moveit_pkg), "config", "ros2_controllers.yaml"])

    # ==========================================
    # 4. DEFINE NODES
    # ==========================================
    
    # A. Custom Evdev Driver (Reads your Joystick Hardware)
    joy_driver_node = Node(
        package=description_pkg,
        executable='evdev_joy',
        output='screen'
    )
    
    # B. Teleop Twist Joy (Converts Joy -> Twist for Servo)
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[
                joy_config_path, 
                {'publish_stamped_twist': True},
                {'frame': 'base_link'}  # <--- THIS IS THE FIX
            ],
        remappings=[('/cmd_vel', '/servo_node/delta_twist_cmds')]
    )

    # C. MoveIt Servo Node (Calculates Inverse Kinematics)
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        parameters=[servo_params, robot_description, robot_description_semantic, robot_description_kinematics],
        output='screen',
    )

    # D. RViz (Visualization)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_path],
        parameters=[robot_description, robot_description_semantic, robot_description_kinematics]
    )

    # E. Robot State Publisher (Publishes TF frames)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )
    
    # F. Controller Manager (Loads the ros2_controllers.yaml)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path],
        output='screen'
    )

    # ==========================================
    # 5. CONTROLLER SPAWNERS (CRITICAL FIX)
    # ==========================================
    # These "turn on" the controllers defined in your YAML

    # Spawns 'joint_state_broadcaster' (Publishers joint angles to TF)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawns 'arm_controller' (Listens to Servo commands)
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    # Spawns 'gripper_controller' (Enables gripper trajectory execution)
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    start_servo_event = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/servo_node/start_servo', 'std_srvs/srv/Trigger', '{}'],
        output='screen'
    )

    # Delay RViz start until the robot is ready (Clean startup)
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay the start command until controller startup has begun.
    delayed_start_servo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[start_servo_event],
        )
    )

    return LaunchDescription([
        joy_driver_node,
        teleop_node,
        servo_node,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        delay_rviz,
        delayed_start_servo
    ])