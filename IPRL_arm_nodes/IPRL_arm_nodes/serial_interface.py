import rclpy
from rclpy.node import Node
import serial

from iprl_arm_interfaces.msg import JointValue


class SerialInterface(Node):

    def __init__(self):
        super().__init__('serial_interface')
        self.subscription = self.create_subscription(
            JointValue,
            'joint_values',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # SERIAL CONNECTION
        self.baud_rate = 115200
        self.ser = serial.Serial('/dev/ttyUSB0', self.baud_rate)

    def listener_callback(self, msg):
        # This implementation sends one command over Serial and reads the response
        joint_ID = msg.joint_id # Base, Shoulder, Elbow, Wrist, Roll, Grasp
        value = msg.value
        is_retrieval = msg.is_retrieval

        message_type = "DES_VAL"
        message_data = str(joint_ID)
        if is_retrieval: #TODO: DECIDE WHETHER THIS WOULD BE BETTER AS A SEPARATE MESSAGE TYPE
            # Change to retrieval type code
            message_type = "CUR_ANG"
        else:
            # Add desired joint value
            message_data = message_data + "," + str(value)

        # Send new value over serial
        message = "<" + message_type + ":" + message_data + ">\n"
        self.ser.write(message.encode("utf-8"))

        self.get_logger().info('Sent message: %s' % message)
        response = self.ser.readline().decode("utf-8")
        self.get_logger().info('Response received: %s' % response)
        

def main(args=None):
    rclpy.init(args=args)

    serial_interface = SerialInterface()

    rclpy.spin(serial_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    serial_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()