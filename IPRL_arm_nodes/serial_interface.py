import rclpy
from rclpy.node import Node
import serial

from std_msgs.msg import Float64MultiArray


class SerialInterface(Node):

    def __init__(self):
        super().__init__('serial_interface')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'joint_angles',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.prevAngles = []
        self.safety_value = 20

        # SERIAL CONNECTION
        self.baud_rate = 115200
        self.ser = serial.Serial('/dev/ttyUSB0', self.baud_rate)

    def listener_callback(self, msg):
        # This implementation sends only the angle differences
        angles = msg.data # Shoulder, Elbow, Wrist, Base, Roll
        angle_diffs = [0,0,0,0,0]
        if (self.prevAngles != []):
            for i in range (0, len(angles)):
                angle_diffs[i] = angles[i] - self.prevAngles[i]
        self.prevAngles = angles

        # Send angle differences over serial
        for i in range (0, len(angle_diffs)):
            if (abs(angle_diffs[i]) > 0.001)&(abs(angle_diffs[i]) < self.safety_value):
                message = "<MOTOR:" + str(i) + "|" + str(angle_diffs[i]) + ">"
                self.ser.write(message.encode("utf-8"))

                self.get_logger().info('Sent message: %s' % message)
        

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