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
HW_CODE_DEADMAN    = 309  # Deadman button (digital)
HW_CODE_BASE_LEFT  = 304  # D-pad / button: rotate base left
HW_CODE_BASE_RIGHT = 306  # D-pad / button: rotate base right

# Standard event codes
HW_CODE_Y_AXIS = 1   # Left Stick Vertical  (forward/back)
HW_CODE_Z_AXIS = 2   # Right Stick Vertical (up/down, confirmed: forward=up)

# 2. ROS MAPPING (Must match xbox_mapping.yaml)
# ------------------------------------------
# Your YAML says: enable_button: 5
ROS_BTN_IDX_DEADMAN = 5

# Your YAML says: x:1, z:4, yaw:3
ROS_AXIS_IDX_X   = 1   # Forward/Back (linear.x)
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

        self._inject_stop = False  # True for one cycle after deadman release
        self._btn_base_left  = False
        self._btn_base_right = False

    def _update_base_yaw(self):
        if self._btn_base_left and not self._btn_base_right:
            self.joy_msg.axes[ROS_AXIS_IDX_YAW] = 1.0
        elif self._btn_base_right and not self._btn_base_left:
            self.joy_msg.axes[ROS_AXIS_IDX_YAW] = -1.0
        else:
            self.joy_msg.axes[ROS_AXIS_IDX_YAW] = 0.0

    def normalize_axis(self, value):
        # Map 0 -> +1.0 and 255 -> -1.0 (forward/up at 0, back/down at 255).
        normalized = (128.0 - value) / 127.5
        return max(-1.0, min(1.0, normalized))

    def loop(self):
        try:
            # Read all pending events from hardware
            for event in self.device.read():

                # --- AXES HANDLING ---
                if event.type == evdev.ecodes.EV_ABS:
                    val_norm = self.normalize_axis(event.value)

                    if event.code == HW_CODE_Y_AXIS:
                        self.joy_msg.axes[ROS_AXIS_IDX_X] = val_norm

                    elif event.code == HW_CODE_Z_AXIS:
                        self.joy_msg.axes[ROS_AXIS_IDX_Z] = val_norm

                # --- BUTTON HANDLING ---
                elif event.type == evdev.ecodes.EV_KEY:
                    if event.code == HW_CODE_DEADMAN:
                        was_held = (self.joy_msg.buttons[ROS_BTN_IDX_DEADMAN] == 1)
                        if event.value == 0 and was_held:
                            # Schedule a zero-velocity injection: one extra publish
                            # cycle with axes=0 and deadman still held so teleop
                            # sends an explicit stop command before going silent.
                            self._inject_stop = True
                        self.joy_msg.buttons[ROS_BTN_IDX_DEADMAN] = event.value
                    elif event.code == HW_CODE_BASE_LEFT:
                        self._btn_base_left = (event.value == 1)
                    elif event.code == HW_CODE_BASE_RIGHT:
                        self._btn_base_right = (event.value == 1)

        except BlockingIOError:
            pass  # No new data this loop

        # On deadman release: publish one zero-velocity message with deadman still
        # held so teleop_twist_joy emits an explicit stop before going silent.
        self._update_base_yaw()

        if self._inject_stop:
            stop_msg = Joy()
            stop_msg.axes    = [0.0] * len(self.joy_msg.axes)
            stop_msg.buttons = [0]   * len(self.joy_msg.buttons)
            stop_msg.buttons[ROS_BTN_IDX_DEADMAN] = 1
            stop_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(stop_msg)
            self._inject_stop = False
            return  # real msg (buttons[5]=0) publishes next cycle

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