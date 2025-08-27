import rclpy
from rclpy.node import Node
import math
import numpy as np
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Transform
from tf2_msgs.msg import TFMessage

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class ArmVisualiser(Node):
    def __init__(self):
        super().__init__('arm_visualiser')

        # Init tf publisher
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_publisher = TransformBroadcaster(self)
        self.timer_frequency = 15  # Hz
        self.timer = self.create_timer(1/self.timer_frequency, self.timer_callback)

        # Debugging publishers
        self.debug_mode = False
        if self.debug_mode:
            self.debug_publisher = self.create_publisher(JointState, 'read_joint_values', 2)
        
        # Init jointstate subscriber
        self.subscription = self.create_subscription(
            JointState,
            "set_joint_values", #or read joint values
            self.listener_callback,
            2)
        self.subscription  # prevent unused variable warning

        # Arm vars
        self.joint_names = ["map","base","shoulder","elbow","wrist","roll","grasp"]
        self.joint_states = [0, -90, 90, 0, 0, 0] # theta1, theta2, theta3, theta4, theta5
        self.link_lengths = [0, 0.1, 0.325, 0.33, 0.195, 0, 0] # in m

    def make_transforms(self, transformation):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = transformation[0]
        t.child_frame_id = transformation[1]

        t.transform.translation.x = float(transformation[2])
        t.transform.translation.y = float(transformation[3])
        t.transform.translation.z = float(transformation[4])
        quat = quaternion_from_euler(
            float(transformation[5]), float(transformation[6]), float(transformation[7]))
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        return t

    def listener_callback(self, msg):
        joints_changed = msg.name
        new_angles = msg.position

        # Update joint states
        for i in range(0, len(joints_changed)):
            self.joint_states[self.joint_names.index(joints_changed[i])-1] = new_angles[i]

    def timer_callback(self):
        msg_list = []

        # Convert from names and positions to transforms for current state
        for joint_id in range (0, len(self.joint_names)-1):

            # Convert from joint angle to tf2
            transform_list = [
                self.joint_names[joint_id], #frame id
                self.joint_names[joint_id+1], #child frame id
                self.link_lengths[joint_id], #x
                0, #y
                0, #z
                0, #rot about x
                0, #rot about y
                0 #rot about z
            ]
            if (joint_id == 1): #shoulder has rotation and height offset
                transform_list[2] = 0
                transform_list[4] = self.link_lengths[joint_id]
                transform_list[5] = math.radians(-90)
                transform_list[6] = math.radians(self.joint_states[joint_id])
            elif (0 <= joint_id <= 4): #theta1-5 use angles
                transform_list[7] = math.radians(self.joint_states[joint_id])

            msg_list.append(self.make_transforms(transform_list))

        self.tf_publisher.sendTransform(msg_list)

        # Pretend to be encoder messages for debugging
        if self.debug_mode:
            msg = JointState()
            msg.name = ["base","shoulder","elbow","wrist"]
            msg.position = self.joint_states[0:4]
            self.debug_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    arm_visualiser = ArmVisualiser()

    rclpy.spin(arm_visualiser)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm_visualiser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()