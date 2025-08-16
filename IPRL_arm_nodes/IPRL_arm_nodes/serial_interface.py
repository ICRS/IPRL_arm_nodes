import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import JointState
import re
PATTERN_ANG = re.compile(r"<CUR_ANG:([-+]?\d*\.\d+),([-+]?\d*\.\d+),([-+]?\d*\.\d+),([-+]?\d*\.\d+)>")


class SerialInterface(Node):

    def __init__(self):
        super().__init__('serial_interface')

        self.publisher = self.create_publisher(JointState, 'read_joint_values', 3)
        pub_timer_freq = 20  # Hz
        self.serial_read_timer = self.create_timer(1/pub_timer_freq, self.read_loop)

        self.subscription = self.create_subscription(
            JointState,
            'set_joint_values',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # SERIAL CONNECTION
        self.baud_rate = 115200
        self.ser = serial.Serial('/dev/ttyUSB0', self.baud_rate) #TODO: CHANGE BACK TO ACM0

    def read_loop(self):
        buffer = ""
        while rclpy.ok():
            if self.ser.in_waiting:
                buffer += self.ser.read(self.ser.in_waiting).decode(errors='ignore')
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self.parse_line(line.strip())
        
    def parse_line(self, line):
        if m := PATTERN_ANG.match(line):

            msg = JointState()
            msg.name = ["base","shoulder","elbow","wrist"]
            msg.position = [float(m.group(1)),float(m.group(2)),float(m.group(3)),float(m.group(4))]
            self.publisher.publish(msg)

        else:
            self.get_logger().warn(f"Unknown line: {line}")

    def listener_callback(self, msg:JointState):
        # This implementation sends one command over Serial and reads the response
        joint_names = msg.name
        joint_values = msg.position
        for i in range(len(joint_names)):
            if joint_names[i] == "base":
                joint_id = 0
            elif joint_names[i]  == "shoulder":
                joint_id = 1
            elif joint_names[i]  == "elbow":
                joint_id = 2
            elif joint_names[i]  == "wrist":
                joint_id = 3
            else:
                self.get_logger().warn(f"Unknown joint: {msg.name}")
            value = joint_values[i]


            message = f"<DES_VAL:{joint_id},{round(value,1)}>\n"
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