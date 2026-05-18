import evdev
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# ==========================================
# CONFIGURATION SECTION
# ==========================================

# 1. HARDWARE CODES (From 'evtest')
# ------------------------------------------
DEVICE_PATH = '/dev/input/event0'

# You confirmed these:
# Deadman can be a button (EV_KEY) or an axis (EV_ABS).
HW_CODE_DEADMAN_BTN = 309  # Example: BTN_TR. Set to None if using axis.
HW_CODE_DEADMAN_AXIS = None  # Example: ABS_RZ. Set to axis code for trigger.
DEADMAN_AXIS_THRESHOLD = 0.5  # 0.0-1.0
HW_CODE_Z_AXIS  = 5    # Your Z Axis (Right Stick Vertical?)

# Standard event codes (Verify these if X/Y move wrongly)
HW_CODE_X_AXIS   = 0   # Left Stick Horizontal
HW_CODE_Y_AXIS   = 1   # Left Stick Vertical
HW_CODE_YAW_AXIS = 2   # Right Stick Horizontal (Standard for Xbox)

# 2. ROS MAPPING (Must match xbox_mapping.yaml)
# ------------------------------------------
# Your YAML says: enable_button: 5
ROS_BTN_IDX_DEADMAN = 5 

# Your YAML says: x:1, y:0, z:4, yaw:3
ROS_AXIS_IDX_X   = 1   # Forward/Back (linear.x)
ROS_AXIS_IDX_Y   = 0   # Left/Right (linear.y)
ROS_AXIS_IDX_Z   = 4   # Up/Down (linear.z)
ROS_AXIS_IDX_YAW = 3   # Rotate (angular.yaw)

class EvdevJoyNode(Node):
    def __init__(self):
        super().__init__('evdev_joy')
        # Publish to /joy so teleop_twist_joy_node can read it
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)
        
        try:
            self.device = evdev.InputDevice(DEVICE_PATH)
            self.get_logger().info(f"Connected to {self.device.name}")
        except FileNotFoundError:
            self.get_logger().error(f"Could not find {DEVICE_PATH}. Is USBIPD attached?")
            exit(1)

        self.deadman_axis_min = None
        self.deadman_axis_max = None
        self.deadman_use_axis = HW_CODE_DEADMAN_AXIS is not None
        if HW_CODE_DEADMAN_AXIS is not None:
            try:
                absinfo = self.device.absinfo(HW_CODE_DEADMAN_AXIS)
                self.deadman_axis_min = absinfo.min
                self.deadman_axis_max = absinfo.max
            except Exception as exc:
                self.get_logger().warn(f"Deadman axis absinfo unavailable: {exc}")

        self.timer = self.create_timer(0.015, self.loop) # 100Hz
        
        # Initialize Joy message with enough slots
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8    # Create 8 axes slots
        self.joy_msg.buttons = [0] * 12  # Create 12 button slots

    def normalize_axis(self, value):
        # Your controller is 0 to 255, with 128 as center
        # We want -1.0 to 1.0
        # (128 - value) / 128.0  -> This inverts it so Up (low value) becomes Positive
        return (128.0 - value) / 128.0

    def normalize_trigger(self, value):
        if self.deadman_axis_min is None or self.deadman_axis_max is None:
            return 0.0
        if self.deadman_axis_max <= self.deadman_axis_min:
            return 0.0
        clamped = max(self.deadman_axis_min, min(self.deadman_axis_max, value))
        return (clamped - self.deadman_axis_min) / (self.deadman_axis_max - self.deadman_axis_min)

    def loop(self):
        try:
            # Read all pending events from hardware
            for event in self.device.read():
                
                # --- AXES HANDLING ---
                if event.type == evdev.ecodes.EV_ABS:
                    if HW_CODE_DEADMAN_AXIS is not None and event.code == HW_CODE_DEADMAN_AXIS:
                        trigger_norm = self.normalize_trigger(event.value)
                        self.joy_msg.buttons[ROS_BTN_IDX_DEADMAN] = 1 if trigger_norm >= DEADMAN_AXIS_THRESHOLD else 0
                        continue

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

                # --- BUTTON HANDLING ---
                elif event.type == evdev.ecodes.EV_KEY:
                    if HW_CODE_DEADMAN_BTN is not None and event.code == HW_CODE_DEADMAN_BTN:
                        # Map the deadman button to ROS Button 5
                        self.joy_msg.buttons[ROS_BTN_IDX_DEADMAN] = event.value

            if not self.deadman_use_axis and HW_CODE_DEADMAN_BTN is not None:
                try:
                    active_keys = self.device.active_keys()
                except Exception:
                    active_keys = []
                self.joy_msg.buttons[ROS_BTN_IDX_DEADMAN] = 1 if HW_CODE_DEADMAN_BTN in active_keys else 0

        except BlockingIOError:
            pass # No new data this loop
            
        # Stamp and publish
        self.joy_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joy_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EvdevJoyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()