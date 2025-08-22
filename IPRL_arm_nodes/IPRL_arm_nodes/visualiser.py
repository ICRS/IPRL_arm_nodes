import rclpy
from rclpy.node import Node
import math
import numpy as np
import matplotlib.pyplot as plt

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
        self.publisher_ = self.create_publisher(TFMessage, 'tf', 2)
        self.timer_period = 0.2  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.subscription = self.create_subscription(
            JointState,
            'set_joint_values', #or read joint values
            self.listener_callback,
            2)
        self.subscription  # prevent unused variable warning

        # Arm vars
        self.joint_names = ['map','base','shoulder','elbow','wrist','roll','grasp','']
        self.joint_defaults = [0, 0, -90, 90, 0, 0, 0]
        self.link_lengths = [0, 0, 0.325, 0.330, 0.195, 0, 0]
        # Holds current state to publish
        self.current_arm_state = TFMessage()

        # # Init map
        map = TFMessage()
        map.transforms = [self.make_transforms(['map','base',0,0,0,0,0,0])]
        self.publisher_.publish(map)
        # Init to home
        init_msg = JointState()
        init_msg.name = self.joint_names.copy()[:-1]
        init_msg.position = self.joint_defaults.copy()
        self.listener_callback(init_msg)

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

        self.get_logger().info("Successfully made transform for %s" % str(transformation[0]))
        self.get_logger().info("")

        return t

    def listener_callback(self, msg):
        joints_changed = msg.name
        new_angles = msg.position

        self.get_logger().info("Processing changed joints: %s" % str(joints_changed))

        transform_msgs = [] 

        # Convert from names and positions to transforms
        for i in range (0, len(joints_changed)):
            joint = joints_changed[i]
            joint_angle = new_angles[i]
            joint_id = self.joint_names.index(joint)

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
            if (1 <= joint_id <= 4): #base shoulder elbow wrist use angles
                transform_list[7] = math.radians(joint_angle)
                if (joint_id == 1): #base has rotation
                    transform_list[5] = math.radians(90)

            self.get_logger().info("Converting joint %s with ID %s" % (str(joint), str(joint_id)))
            self.get_logger().info("Transform_list: %s" % str(transform_list))

            transform_msgs.append(self.make_transforms(transform_list))

        self.current_arm_state.transforms = transform_msgs
    
    def timer_callback(self):
        self.publisher_.publish(self.current_arm_state)


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