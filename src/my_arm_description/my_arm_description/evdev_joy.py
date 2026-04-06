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
HW_CODE_DEADMAN = 309  # Your Deadman Button RIGHT TRIGGER
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

        self.timer = self.create_timer(0.02, self.loop) # 50Hz
        
        # Initialize Joy message with enough slots
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8    # Create 8 axes slots
        self.joy_msg.buttons = [0] * 12  # Create 12 button slots

    def normalize_axis(self, value):
        # Your controller is 0 to 255, with 128 as center
        # We want -1.0 to 1.0
        # (128 - value) / 128.0  -> This inverts it so Up (low value) becomes Positive
        return (128.0 - value) / 128.0

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

                # --- BUTTON HANDLING ---
                elif event.type == evdev.ecodes.EV_KEY:
                    if event.code == HW_CODE_DEADMAN:
                        # Map hardware 305 to ROS Button 5
                        self.joy_msg.buttons[ROS_BTN_IDX_DEADMAN] = event.value

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