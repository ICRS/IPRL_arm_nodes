import rclpy
from rclpy.node import Node
import serial

from std_msgs.msg import String


class SerialInterface(Node):

    def __init__(self):
        super().__init__('serial_interface')
        self.subscription = self.create_subscription(
            String,
            'joint_angles',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.prevAngles = []

        # SERIAL CONNECTION
        self.ser = serial.Serial('/dev/ttyUSB0')

    def listener_callback(self, msg):
        # This implementation sends only the angle differences
        angles = msg.data # Shoulder, Elbow, Wrist, Base, Roll
        if (self.prevAngles != []):
            angle_diff = []
            for i in range (0, len(msg.data)):
                angle_diff[i] = msg.data[i] - self.prevAngles[i]
        self.prevAngles = msg.data

        # Send angle differences over serial
        for i in range (0, len(angle_diff)):
            if (angle_diff[i] > 0.001):
                message = "<MOTOR:" + str(i) + "|" + str(angle_diff[i]) + ">"
                self.ser.write(message)
        
        self.get_logger().info('Sent angle diffs: %s', str(angle_diff))


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