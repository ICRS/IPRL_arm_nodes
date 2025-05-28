import rclpy
from rclpy.node import Node
import arm_utilities

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64

class arm_IK(Node):
    def __init__(self):
        super().__init__("arm_IK")
        self.IPRL_arm = arm_utilities.Arm([0, -100, 160, -45, 0]) # TODO: replace this with initial encoder pull

        # Define publishers
        self.angles_publisher_ = self.create_publisher(Float64MultiArray, "joint_angles", 10)
        self.gripper_publisher_ = self.create_publisher(Float64, "gripper_percent_closed", 10)
        self.timer_ = self.create_timer(1.0/5.0, self.publish_angles_and_gripper)
        
        # Define subscription
        self.subscription = self.create_subscription(Float64MultiArray, "commands", self.on_receive_command, 10)

    def publish_angles_and_gripper(self):
        angle_array = self.IPRL_arm.getDesiredAngles()
        gripper_percent_closed = float(self.IPRL_arm.getGripperState())

        angle_msg = Float64MultiArray()
        angle_msg.data = [float(item) for item in angle_array]

        gripper_msg = Float64()
        gripper_msg.data = gripper_percent_closed

        self.angles_publisher_.publish(angle_msg)
        self.gripper_publisher_.publish(gripper_msg)

        #self.get_logger().info('Publishing: "%s"' % angle_msg.data)

    def on_receive_command(self, msg):
        """ 
        msg.data format: [dx,dy,dphi,dtheta,droll]
        calculates new set of IK
        """
        self.IPRL_arm.updateCurrentAngles(self.IPRL_arm.getDesiredAngles()) # Sets current angles to desired angles. TODO: change to encoder values
        self.IPRL_arm.calculateDesiredAngles(msg.data[0],msg.data[1],msg.data[2],msg.data[3],msg.data[4])
        self.get_logger().info('Arm is at (x,y): "%s"' % str(self.IPRL_arm.FK2D()))
        

def main(args=None):
    rclpy.init(args=args)
    node = arm_IK()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()