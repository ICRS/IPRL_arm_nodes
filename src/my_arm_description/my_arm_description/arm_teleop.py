"""Joystick -> shoulder/elbow joint speeds for a 2-link planar arm, sent to MoveIt Servo.

The arm (joint_2 shoulder, joint_3 elbow) moves in the base's x-z plane, so the tip
velocity [vx, vz] maps to joint speeds through a 2x2 Jacobian. We solve that here and
send the joint speeds to Servo as JointJog commands; Servo then applies joint limits,
smoothing and its incoming-command timeout.

Nothing is sent unless the deadman is held. If /joy stops, nothing is sent, and Servo
halts the arm after incoming_command_timeout.
"""
import math

import rclpy
from control_msgs.msg import JointJog
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy

# ==========================================
# CONFIGURATION SECTION
# ==========================================
# /joy layout (filled in by evdev_joy.py)
JOY_AXIS_FWD = 1        # left stick forward/back: +1 forward
JOY_AXIS_UP = 4         # right stick up/down:     +1 up
JOY_BTN_DEADMAN = 5     # 1 while held

SHOULDER = 'joint_2'
ELBOW = 'joint_3'

# Link lengths from my_arm.urdf.xacro (shoulder->elbow, elbow->ee_link), metres
UPPER_ARM = 0.405
FOREARM = 0.345


class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        self.tip_speed = self.declare_parameter('tip_speed', 0.08).value        # m/s at full stick
        self.max_joint_speed = self.declare_parameter('max_joint_speed', 0.3).value  # rad/s
        self.damping = self.declare_parameter('damping', 0.03).value            # m, near straight arm
        self.deadzone = self.declare_parameter('deadzone', 0.05).value

        self.q = None            # (shoulder, elbow) in rad, from /joint_states
        self.was_moving = False

        self.pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.create_subscription(Joy, '/joy', self.on_joy, 10)

    def on_joint_states(self, msg):
        try:
            self.q = (msg.position[msg.name.index(SHOULDER)], msg.position[msg.name.index(ELBOW)])
        except ValueError:
            pass

    def on_joy(self, msg):
        held = len(msg.buttons) > JOY_BTN_DEADMAN and msg.buttons[JOY_BTN_DEADMAN] == 1
        fwd = msg.axes[JOY_AXIS_FWD] if len(msg.axes) > JOY_AXIS_FWD else 0.0
        up = msg.axes[JOY_AXIS_UP] if len(msg.axes) > JOY_AXIS_UP else 0.0
        if abs(fwd) < self.deadzone:
            fwd = 0.0
        if abs(up) < self.deadzone:
            up = 0.0

        if not held or self.q is None or (fwd == 0.0 and up == 0.0):
            if self.was_moving:
                self.publish(0.0, 0.0)  # stop immediately instead of waiting for Servo's timeout
                self.was_moving = False
            return

        q_dot = self.joint_speeds(fwd * self.tip_speed, up * self.tip_speed)
        self.publish(*q_dot)
        self.was_moving = True

    def joint_speeds(self, vx, vz):
        """Damped least-squares inverse of the 2-link Jacobian: q_dot = J^T (J J^T + l^2 I)^-1 v."""
        q1, q2 = self.q
        c1, s1 = math.cos(q1), math.sin(q1)
        c12, s12 = math.cos(q1 + q2), math.sin(q1 + q2)
        # Tip: x = L1 sin q1 + L2 sin(q1+q2),  z = L1 cos q1 + L2 cos(q1+q2)
        j11, j12 = UPPER_ARM * c1 + FOREARM * c12, FOREARM * c12
        j21, j22 = -UPPER_ARM * s1 - FOREARM * s12, -FOREARM * s12

        # A = J J^T + l^2 I  (2x2, symmetric)
        lam2 = self.damping ** 2
        a11 = j11 * j11 + j12 * j12 + lam2
        a12 = j11 * j21 + j12 * j22
        a22 = j21 * j21 + j22 * j22 + lam2
        det = a11 * a22 - a12 * a12
        # w = A^-1 v
        w1 = (a22 * vx - a12 * vz) / det
        w2 = (-a12 * vx + a11 * vz) / det
        # q_dot = J^T w
        q1_dot = j11 * w1 + j21 * w2
        q2_dot = j12 * w1 + j22 * w2

        # Keep both joints within their speed limit without changing the direction of motion.
        biggest = max(abs(q1_dot), abs(q2_dot))
        if biggest > self.max_joint_speed:
            scale = self.max_joint_speed / biggest
            q1_dot *= scale
            q2_dot *= scale
        return q1_dot, q2_dot

    def publish(self, shoulder_speed, elbow_speed):
        jog = JointJog()
        jog.header.stamp = self.get_clock().now().to_msg()
        jog.header.frame_id = 'base_link'
        jog.joint_names = [SHOULDER, ELBOW]
        jog.velocities = [shoulder_speed, elbow_speed]
        self.pub.publish(jog)


def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleop()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
