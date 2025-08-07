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
        self.ser = serial.Serial('/dev/ttyACM0', self.baud_rate)

    def listener_callback(self, msg):
        # This implementation sends absolute angles
        angles = msg.data # Shoulder, Elbow, Wrist, Base, Roll
        new_angles = [0,0,0,0,0]
        if (self.prevAngles != []):
            for i in range (0, len(angles)):
                new_angles[i] = angles[i] - self.prevAngles[i]
        self.prevAngles = angles

        # Send angle differences over serial
        for i in range (0, len(new_angles)):
            if (abs(new_angles[i]) > 0.001)&(abs(new_angles[i]) < self.safety_value):
                message = "<MOTOR_" + str(i) + ":" + str(new_angles[i]) + ">\n"
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