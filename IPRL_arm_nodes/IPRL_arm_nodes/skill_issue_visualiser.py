import rclpy
from rclpy.node import Node
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

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
        self.joint_names = ['base','shoulder','elbow','wrist']
        self.joint_states = [45, -90, 90, -45] #theta_1, theta_2, theta_3, theta_4
        self.link_lengths = [10, 32.5, 33.0, 19.5] # base-shoulder, shoulder-elbow, elbow-wrist, wrist-end (in m)

        # Init plot
        plt.ion()
        self.arm_sim_fig = plt.figure()
        self.ax = self.arm_sim_fig.add_subplot(111, projection='3d')
        
        # Init to home
        init_msg = JointState()
        init_msg.name = self.joint_names
        init_msg.position = self.joint_states
        self.listener_callback(init_msg)

        # Start showing plot
        plt.show(block=True)

    def update_joint_points(self):
        theta_1, theta_2, theta_3, theta_4 = math.radians(self.joint_states[0]), math.radians(self.joint_states[1]), math.radians(self.joint_states[2]), math.radians(self.joint_states[3])
        self.get_logger().info("Thetas: %s" % str([theta_1, theta_2, theta_3, theta_4]))
        L1, L2, L3, L4 = self.link_lengths[0], self.link_lengths[1], self.link_lengths[2], self.link_lengths[3]

        c1, c2 = math.cos(theta_1), math.cos(theta_2)
        s1, s2 = math.sin(theta_1), math.sin(theta_2)
        c23, c234 = math.cos(theta_2+theta_3), math.cos(theta_2+theta_3+theta_4)
        s23, s234 = math.sin(theta_2+theta_3), math.sin(theta_2+theta_3+theta_4)

        base_point = [0,0,0]
        shoulder_point = [0,0,L1]
        elbow_point = [L2*c1*c2, L2*s1*c2, L1-L2*s2]
        wrist_point = [c1*(L2*c2+L3*c23), s1*(L2*c2+L3*c23), L1-L2*s2-L3*s23]
        self.get_logger().info("Elbow point: %s" % str(wrist_point))
        end_effector_point = [c1*(L2*c2+L3*c23+L4*c234), s1*(L2*c2+L3*c23+L4*c234), L1-L2*s2-L3*s23-L4*s234]

        return [base_point, shoulder_point, elbow_point, wrist_point, end_effector_point]

    def plot_joint_points(self, point_list):
        self.get_logger().info("Point list: %s" % str(point_list))
        for i in range(0, len(point_list)-1):
            start = point_list[i]
            end = point_list[i+1]
            self.get_logger().info("Plotting from point %s to point %s" % (str(start), str(end)))
            self.ax.plot([start[0], end[0]], [start[1], end[1]], zs=[start[2], end[2]])
        self.arm_sim_fig.canvas.draw()

    def listener_callback(self, msg):
        joints_changed = msg.name
        new_angles = msg.position

        self.get_logger().info("Processing changed joints: %s" % str(joints_changed))

        # Update 
        for i in range (0, len(joints_changed)):
            joint = joints_changed[i]
            joint_angle = new_angles[i]

            if joint in self.joint_names:
                self.joint_states[self.joint_names.index(joint)] = joint_angle
        
        self.plot_joint_points(self.update_joint_points())

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