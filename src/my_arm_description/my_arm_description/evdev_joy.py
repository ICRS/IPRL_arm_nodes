import evdev
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointJog
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ==========================================
# CONFIGURATION SECTION
# ==========================================

# 1. HARDWARE CODES (From 'evtest')
# ------------------------------------------
DEVICE_PATH = '/dev/input/event0'

# You confirmed these:
HW_CODE_DEADMAN = 309  # Your Deadman Button RIGHT BUMPER
HW_CODE_Z_AXIS  = 5    # Your Z Axis (Right Stick Vertical?)

# Standard event codes (Verify these if X/Y move wrongly)
HW_CODE_X_AXIS   = 0   # Left Stick Horizontal
HW_CODE_Y_AXIS   = 1   # Left Stick Vertical
HW_CODE_YAW_AXIS = 2   # Right Stick Horizontal (Standard for Xbox)
HW_CODE_HAT0X    = 16  # D-pad left/right

# Button events requested for gripper control
HW_CODE_BTN_CLOSE_GRIPPER = 305  # BTN_EAST (B)
HW_CODE_BTN_OPEN_GRIPPER  = 307  # BTN_NORTH (Y)

# 2. ROS MAPPING (Must match xbox_mapping.yaml)
# ------------------------------------------
# Your YAML says: enable_button: 5
ROS_BTN_IDX_DEADMAN = 5 

# Your YAML says: x:1, y:0, z:4, yaw:3
ROS_AXIS_IDX_X   = 1   # Forward/Back (linear.x)
ROS_AXIS_IDX_Y   = 0   # Left/Right (linear.y)
ROS_AXIS_IDX_Z   = 4   # Up/Down (linear.z)
ROS_AXIS_IDX_YAW = 3   # Rotate (angular.yaw)
ROS_AXIS_IDX_WRIST = 5 # Wrist rotate (-1 left, +1 right)

ROS_BTN_IDX_CLOSE_GRIPPER = 1  # BTN_EAST (B)
ROS_BTN_IDX_OPEN_GRIPPER  = 3  # BTN_NORTH (Y)

# 3. DIRECT COMMAND TOPICS (for wrist/gripper actuation)
# ------------------------------------------
WRIST_JOINT_NAME = 'joint_5'
WRIST_JOG_SPEED = 1.0  # Positive rotates right, negative rotates left.

GRIPPER_JOINT_NAME = 'gripper_joint'
GRIPPER_OPEN_POS = 0.03
GRIPPER_CLOSED_POS = 0.0
GRIPPER_MOVE_TIME_SEC = 0.25

class EvdevJoyNode(Node):
    def __init__(self):
        super().__init__('evdev_joy')
        # Publish to /joy so teleop_twist_joy_node can read it
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)
        self.joint_jog_pub_ = self.create_publisher(JointJog, '/delta_joint_cmds', 10)
        self.gripper_action_client_ = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )
        
        try:
            self.device = evdev.InputDevice(DEVICE_PATH)
            self.get_logger().info(f"Connected to {self.device.name}")
        except FileNotFoundError:
            self.get_logger().error(f"Could not find {DEVICE_PATH}. Is USBIPD attached?")
            exit(1)

        self.timer = self.create_timer(0.02, self.loop) # 50Hz
        
        # Initialize Joy message with enough slots
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8    # Create 8 axes slots
        self.joy_msg.buttons = [0] * 12  # Create 12 button slots
        self.wrist_direction = 0

    def normalize_axis(self, value):
        # Your controller is 0 to 255, with 128 as center
        # We want -1.0 to 1.0
        # (128 - value) / 128.0  -> This inverts it so Up (low value) becomes Positive
        return (128.0 - value) / 128.0

    def publish_wrist_jog(self):
        if self.wrist_direction == 0:
            return

        cmd = JointJog()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.joint_names = [WRIST_JOINT_NAME]
        cmd.velocities = [WRIST_JOG_SPEED * float(self.wrist_direction)]
        cmd.duration = 0.1
        self.joint_jog_pub_.publish(cmd)

    def publish_gripper_target(self, target_position):
        if not self.gripper_action_client_.server_is_ready():
            self.get_logger().warn('Gripper action server not ready yet, command ignored.')
            return

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = [GRIPPER_JOINT_NAME]

        point = JointTrajectoryPoint()
        point.positions = [float(target_position)]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(GRIPPER_MOVE_TIME_SEC * 1e9)
        traj.points = [point]

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj
        self.gripper_action_client_.send_goal_async(goal_msg)

    def loop(self):
        try:
            # Read all pending events from hardware
            for event in self.device.read():
                
                # --- AXES HANDLING ---
                if event.type == evdev.ecodes.EV_ABS:
                    val_norm = self.normalize_axis(event.value)

                    if event.code == HW_CODE_X_AXIS:
                        # Map hardware X to ROS Axis 0 (Left/Right)
                        # Note: Usually needs sign flip depending on preference
                        self.joy_msg.axes[ROS_AXIS_IDX_Y] = -val_norm 
                        
                    elif event.code == HW_CODE_Y_AXIS:
                        # Map hardware Y to ROS Axis 1 (Forward/Back)
                        self.joy_msg.axes[ROS_AXIS_IDX_X] = val_norm 

                    elif event.code == HW_CODE_Z_AXIS:
                        # Map hardware Z (5) to ROS Axis 4
                        self.joy_msg.axes[ROS_AXIS_IDX_Z] = val_norm

                    elif event.code == HW_CODE_YAW_AXIS:
                        # Map hardware Yaw to ROS Axis 3
                        self.joy_msg.axes[ROS_AXIS_IDX_YAW] = -val_norm

                    elif event.code == HW_CODE_HAT0X:
                        # D-pad left/right controls wrist rotation.
                        # Requested mapping: -1 => left, +1 => right.
                        if event.value in (-1, 0, 1):
                            self.joy_msg.axes[ROS_AXIS_IDX_WRIST] = float(event.value)
                            self.wrist_direction = int(event.value)

                # --- BUTTON HANDLING ---
                elif event.type == evdev.ecodes.EV_KEY:
                    if event.code == HW_CODE_DEADMAN:
                        # Map deadman to ROS Button 5
                        self.joy_msg.buttons[ROS_BTN_IDX_DEADMAN] = event.value

                    elif event.code == HW_CODE_BTN_CLOSE_GRIPPER:
                        # Close gripper on press.
                        self.joy_msg.buttons[ROS_BTN_IDX_CLOSE_GRIPPER] = 1 if event.value == 1 else 0
                        if event.value == 1:
                            self.publish_gripper_target(GRIPPER_CLOSED_POS)

                    elif event.code == HW_CODE_BTN_OPEN_GRIPPER:
                        # Open gripper on press.
                        self.joy_msg.buttons[ROS_BTN_IDX_OPEN_GRIPPER] = 1 if event.value == 1 else 0
                        if event.value == 1:
                            self.publish_gripper_target(GRIPPER_OPEN_POS)

        except BlockingIOError:
            pass # No new data this loop
            
        # Stamp and publish
        self.joy_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joy_msg)
        self.publish_wrist_jog()

def main(args=None):
    rclpy.init(args=args)
    node = EvdevJoyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()