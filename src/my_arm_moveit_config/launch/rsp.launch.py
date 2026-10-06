from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    # demo.launch.py includes this file, so the robot description published here is the one
    # ros2_control_node loads. Forward use_mock_hardware so mock vs ESP32 hardware is honoured.
    moveit_config = (
        MoveItConfigsBuilder("my_arm", package_name="my_arm_moveit_config")
        .robot_description(
            mappings={"use_mock_hardware": LaunchConfiguration("use_mock_hardware")}
        )
        .to_moveit_configs()
    )

    ld = generate_rsp_launch(moveit_config)
    ld.entities.insert(
        0,
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Use mock_components/GenericSystem instead of the real ESP32 serial hardware",
        ),
    )
    return ld
