import rclpy
from rclpy.node import Node
import math
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import JointState

class ArmVisualiser(Node):
    def __init__(self):
        super().__init__('arm_visualiser')
        
        self.subscription = self.create_subscription(
            JointState,
            'set_joint_values', #or read joint values
            self.listener_callback,
            2)
        self.subscription  # prevent unused variable warning

        # Arm vars
        self.joint_names = ['base','shoulder','elbow','wrist','roll','grasp','']
        self.joint_defaults = [45, -90, 90, 45, 0, 0]
        self.link_lengths = [0, 0.325, 0.330, 0.195, 0, 0]

        # Init plot
        self.arm_sim = plt.figure()
        self.ax = self.arm_sim.add_subplot(111, projection='3d')

        # Init to home
        init_msg = JointState()
        init_msg.name = self.joint_names.copy()[:-1]
        init_msg.position = self.joint_defaults.copy()
        self.listener_callback(init_msg)

        # Start showing plot
        plt.show()

    def make_relative_point(self, transformation):
        vector = transformation[2:5]
        # Rotate about x
        theta = transformation[5]
        vector = [vector[0],
                  vector[1]*math.cos(theta) - vector[2]*math.sin(theta),
                  vector[1]*math.sin(theta) + vector[2]*math.cos(theta)]
        # Rotate about y
        phi = transformation[6]
        vector = [vector[0]*math.cos(phi) - vector[2]*math.sin(phi),
                  vector[1],
                  vector[0]*math.sin(phi) + vector[2]*math.cos(phi)]
        # Rotate about z
        gamma = transformation[7]
        vector = [vector[0]*math.cos(gamma) - vector[1]*math.sin(gamma),
                  vector[0]*math.sin(gamma) + vector[1]*math.cos(gamma),
                  vector[2]]
        return vector

    def plot_relative_points(self, point_list):
        self.get_logger().info("Point list: %s" % str(point_list))
        plt.cla()
        x, y, z = 0, 0, 0
        x_e, y_e, z_e = 0, 0, 0
        for point in point_list:
            x_e += point[0]
            y_e += point[1]
            z_e += point[2]
            self.ax.plot([x, x_e], [y, y_e], zs=[z,z_e])
            x,y,z = x_e,y_e,z_e
        plt.draw()

    def listener_callback(self, msg):
        joints_changed = msg.name
        new_angles = msg.position

        self.get_logger().info("Processing changed joints: %s" % str(joints_changed))

        relative_points = [] 

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
                    transform_list[5] = math.radians(-90)

            self.get_logger().info("Converting joint %s with ID %s" % (str(joint), str(joint_id)))

            relative_points.append(self.make_relative_point(transform_list))

        self.plot_relative_points(relative_points)

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