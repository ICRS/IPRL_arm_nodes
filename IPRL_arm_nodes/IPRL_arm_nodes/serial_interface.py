import rclpy
from rclpy.node import Node
import serial

from iprl_arm_interfaces.msg import JointValue


class SerialInterface(Node):

    def __init__(self):
        super().__init__('serial_interface')
        self.received_values_queue = []

        self.publisher = self.create_publisher(JointValue, 'read_joint_values', 3)
        pub_timer_freq = 20  # Hz
        self.pub_timer = self.create_timer(1/pub_timer_freq, self.timer_callback)

        self.subscription = self.create_subscription(
            JointValue,
            'request_joint_values',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # SERIAL CONNECTION
        self.baud_rate = 115200
        self.ser = serial.Serial('/dev/ttyUSB0', self.baud_rate) #TODO: CHANGE BACK TO ACM0

    def timer_callback(self):
        if self.received_values_queue != []:
            value_pair = self.received_values_queue.pop(-1)
            msg = JointValue()
            msg.joint_id = value_pair[0]
            msg.value = value_pair[1]
            msg.is_retrieval = True
            
            self.publisher.publish(msg)

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

        if is_retrieval:
            received_angle = float(response[response.find(":")+1:-1].strip(">\r"))
            self.received_values_queue.append((joint_ID, received_angle))
        

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