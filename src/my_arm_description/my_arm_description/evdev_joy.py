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

HW_CODE_DEADMAN   = 309  # Deadman button (digital). MUST be held to move.
HW_CODE_FWD_BACK  = 1    # EV_ABS, 0-255. Forward/back.  <128 = forward, >128 = back.
HW_CODE_UP_DOWN   = 5    # EV_ABS, 0-255. Up/down.       <128 = up,      >128 = down.
HW_CODE_BASE_HAT  = 16   # EV_ABS HAT0X, values -1/0/+1. Arrow pad: -1 = left, +1 = right.

# 2. ROS MAPPING (Must match xbox_mapping.yaml)
# ------------------------------------------
ROS_BTN_IDX_DEADMAN = 5   # enable_button: 5
ROS_AXIS_IDX_X      = 1   # axis_linear.x  -> Forward/back (linear.x)
ROS_AXIS_IDX_Z      = 4   # axis_linear.z  -> Up/down      (linear.z)
ROS_AXIS_IDX_YAW    = 3   # axis_angular.yaw -> Rotate base (angular.yaw)

# 3. DIRECTION / POLARITY (flip in one line if a motion is reversed)
# ------------------------------------------
# Deadman is inverted in hardware: physically holding 309 emits value 0.
# So "held" == value 0. Set to 1 if you ever swap to a normal-polarity button.
DEADMAN_HELD_VALUE = 0

DIR_FWD  = +1.0   # base_link +x = forward. normalize() already gives + for <128.
DIR_UP   = +1.0   # base_link +z = up.
DIR_BASE = +1.0   # HAT left (-1) -> +yaw (left). Flip to -1.0 to reverse base rotation.

DEADZONE = 0.05   # ignore tiny stick-center drift to avoid creep/oscillation.


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

        self.timer = self.create_timer(0.02, self.loop)  # 50Hz

        # Initialize Joy message with enough slots
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8    # Create 8 axes slots
        self.joy_msg.buttons = [0] * 12  # Create 12 button slots

    def normalize_axis(self, value):
        # Map 0 -> +1.0 and 255 -> -1.0 (forward/up at 0, back/down at 255).
        normalized = (128.0 - value) / 127.5
        normalized = max(-1.0, min(1.0, normalized))
        if abs(normalized) < DEADZONE:
            return 0.0
        return normalized

    def loop(self):
        try:
            # Read all pending events from hardware
            for event in self.device.read():

                # --- AXES HANDLING ---
                if event.type == evdev.ecodes.EV_ABS:
                    if event.code == HW_CODE_FWD_BACK:
                        self.joy_msg.axes[ROS_AXIS_IDX_X] = DIR_FWD * self.normalize_axis(event.value)

                    elif event.code == HW_CODE_UP_DOWN:
                        self.joy_msg.axes[ROS_AXIS_IDX_Z] = DIR_UP * self.normalize_axis(event.value)

                    elif event.code == HW_CODE_BASE_HAT:
                        # HAT0X is already -1/0/+1, so do NOT normalize it.
                        self.joy_msg.axes[ROS_AXIS_IDX_YAW] = DIR_BASE * (-float(event.value))

                # --- BUTTON HANDLING ---
                elif event.type == evdev.ecodes.EV_KEY:
                    if event.code == HW_CODE_DEADMAN:
                        held = (event.value == DEADMAN_HELD_VALUE)
                        self.joy_msg.buttons[ROS_BTN_IDX_DEADMAN] = 1 if held else 0

        except BlockingIOError:
            pass  # No new data this loop

        # Stamp and publish. On deadman release buttons[5] is 0, so teleop_twist_joy
        # emits one zero-twist on the enable->disable transition and Servo's
        # incoming_command_timeout halts the arm.
        self.joy_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joy_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EvdevJoyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
