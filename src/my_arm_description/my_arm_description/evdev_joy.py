import sys

import evdev
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

# ==========================================
# CONFIGURATION SECTION
# ==========================================

# 1. HARDWARE CODES (PowerA NSW wired controller, measured with joy_map.py / evtest)
# ------------------------------------------
# Stable path for this pad; override with:  --ros-args -p device:=/dev/input/eventN
DEFAULT_DEVICE = '/dev/input/by-id/usb-PowerA_NSW_wired_controller-event-joystick'

HW_CODE_DEADMAN   = 309  # BTN_Z. MUST be held to move. Reads 1 while pressed.
HW_CODE_FWD_BACK  = 1    # ABS_Y  left stick,  0-255. 0 = forward, 255 = back.
HW_CODE_BASE      = 0    # ABS_X  left stick,  0-255. 0 = left,    255 = right.
HW_CODE_UP_DOWN   = 5    # ABS_RZ right stick, 0-255. 0 = up,      255 = down.

# 2. ROS MAPPING (Must match the JOY_* constants in arm_teleop.py)
# ------------------------------------------
ROS_BTN_IDX_DEADMAN = 5   # JOY_BTN_DEADMAN
ROS_AXIS_IDX_X      = 1   # JOY_AXIS_FWD -> arm forward/back
ROS_AXIS_IDX_Z      = 4   # JOY_AXIS_UP  -> arm up/down

# Base is not part of the arm kinematics: it is driven directly by publishing a
# direction (+1 right, -1 left, 0 stop). The hardware plugin stops the base if this
# topic goes quiet for 200 ms (e.g. this node dies).
BASE_TOPIC = '/base_cmd'

# 3. DIRECTION / POLARITY (flip in one line if a motion is reversed)
# ------------------------------------------
DEADMAN_HELD_VALUE = 1    # this pad reports 1 while the deadman button is held

DIR_FWD  = -1.0   # flipped after real-arm test: stick forward must move the arm forward.
DIR_UP   = +1.0   # base_link +z = up.      normalize() already gives + for 0 (up).
DIR_BASE = -1.0   # normalize() gives - for 255 (right); -1 makes right = +1 (turn right).

DEADZONE = 0.05        # ignore tiny stick-center drift to avoid creep/oscillation.
BASE_THRESHOLD = 0.5   # stick must be pushed past half way before the base turns.


class EvdevJoyNode(Node):
    def __init__(self):
        super().__init__('evdev_joy')
        # Publish to /joy so arm_teleop can read it
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)
        self.base_publisher_ = self.create_publisher(Float64, BASE_TOPIC, 10)

        device_path = self.declare_parameter('device', DEFAULT_DEVICE).value
        try:
            self.device = evdev.InputDevice(device_path)
            self.get_logger().info(f"Connected to {self.device.name} ({device_path})")
        except FileNotFoundError:
            self.get_logger().error(f"Could not find {device_path}. Is the controller plugged in?")
            sys.exit(1)
        except PermissionError:
            self.get_logger().error(
                f"No permission to read {device_path}. Add yourself to the 'input' group "
                "(sudo usermod -aG input $USER) and log out/in.")
            sys.exit(1)

        self.timer = self.create_timer(0.02, self.loop)  # 50Hz

        # Initialize Joy message with enough slots
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8    # Create 8 axes slots
        self.joy_msg.buttons = [0] * 12  # Create 12 button slots

        self.base_axis = 0.0

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

                    elif event.code == HW_CODE_BASE:
                        self.base_axis = DIR_BASE * self.normalize_axis(event.value)

                # --- BUTTON HANDLING ---
                elif event.type == evdev.ecodes.EV_KEY:
                    if event.code == HW_CODE_DEADMAN:
                        held = (event.value == DEADMAN_HELD_VALUE)
                        self.joy_msg.buttons[ROS_BTN_IDX_DEADMAN] = 1 if held else 0

        except BlockingIOError:
            pass  # No new data this loop

        # Stamp and publish. On deadman release buttons[5] is 0, so arm_teleop sends one
        # zero command and then nothing; Servo's incoming_command_timeout halts the arm.
        self.joy_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joy_msg)

        # Base direction: only while the deadman is held and the stick is pushed far enough.
        base_dir = 0.0
        if self.joy_msg.buttons[ROS_BTN_IDX_DEADMAN] and abs(self.base_axis) >= BASE_THRESHOLD:
            base_dir = 1.0 if self.base_axis > 0.0 else -1.0
        self.base_publisher_.publish(Float64(data=base_dir))


def main(args=None):
    rclpy.init(args=args)
    node = EvdevJoyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
