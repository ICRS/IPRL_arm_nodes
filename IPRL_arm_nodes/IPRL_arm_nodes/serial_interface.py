import rclpy
from rclpy.node import Node
import serial
import glob
import time
import re
import threading
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Bool


PATTERN_ANG = re.compile(r"<CUR_ANG:([-+]?\d*\.\d+),([-+]?\d*\.\d+),([-+]?\d*\.\d+),([-+]?\d*\.\d+)>")
PATTERN_PH = re.compile(r"<PH_PROBE:(\d+)>")
PATTERN_PONG = re.compile(r"<PONG:(\d+)>")


def adc_to_ph(adc_reading: int):
    """This function maps the 16-bit unsigned pH sensor ADC reading to an actual pH between 1 and 14"""
    MAX_PH = 14.0
    MIN_PH = 1.0
    return float(adc_reading) * ((MAX_PH - MIN_PH) / (2**16-1)) + MIN_PH



class SerialInterface(Node):

    def __init__(self):
        super().__init__('serial_interface')

        self.publisher = self.create_publisher(JointState, "read_joint_values", 2)

        self.subscription = self.create_subscription(
            JointState,
            "set_joint_values",
            self.listener_callback,
            10)
        
        self.ph_sub = self.create_subscription(Bool, "ph/request", self.ph_request, 10)
        self.ph_pub = self.create_publisher(Float32, "ph/raw", 10)

        # Setup serial
        ports = sorted(glob.glob('/dev/ttyUSB*'))
        if not ports:
            self.get_logger().error("No /dev/ttyACM* devices found.")
            raise RuntimeError("No ACM serial device found.")
        self.ser = None

        # Get the port
        connected = False
        self.baud_rate = 115200
        while not connected:
            for port in ports:
                try:
                    self.get_logger().info(f"Trying serial port: {port}")
                    self.ser = serial.Serial('/dev/ttyUSB0', self.baud_rate) #TODO: CHANGE BACK TO ACM0
                    time.sleep(2)  # give device time to reset if needed
                    self.get_logger().info(f"Connected to {port}")
                except serial.SerialException as e:
                    self.get_logger().warn(f"Failed to open {port}: {e}")

                # Check the serial port is okay
                if self.ser is None:
                    self.get_logger().info("Couldn't get serial port")
                    time.sleep(1)
                    continue

                # Ping the device to check it responds and has the correct ID
                connected = self.serial_ping()

                # Invalid device: close the connection and try the next device
                if not connected:
                    self.ser.close()
                    continue

                # Valid device: move on
                else:
                    self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
                    self.read_thread.start()
                    self.get_logger().info("Started reading thread")
                    break

    def serial_ping(self) -> bool:

        buffer = ""
        start_time = time.time()
        pong_received = False
        while not pong_received:

            # Timeout
            if (time.time() > (start_time + 3.0)) and not pong_received:
                break

            # Send PING
            out = "<PING:1>\n"
            out_bytes = bytes(out.encode("utf-8"))
            try:
                self.ser.write(out_bytes)
                self.get_logger().info("Sent PING")
            except serial.serialutil.SerialException:
                continue

            time.sleep(0.2)

            # Wait for PONG
            if self.ser.in_waiting:
                buffer += self.ser.read(self.ser.in_waiting).decode(errors='ignore')
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()

                    # Check if the message is a PONG
                    m = PATTERN_PONG.match(line)
                    if m is not None:

                        device_id = int(m.group(1))

                        # PONG contained invalid ID
                        if device_id != 50:
                            self.get_logger().info(f"Received PONG, incorrect ID: '{device_id}'")
                            return pong_received

                        # PONG containted correct ID
                        else:
                            pong_received = True
                            self.get_logger().info(f"Received PONG, correct ID: '{device_id}'")
                            return pong_received

        return pong_received
    
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

            self.get_logger().info(f"Sent message: {msg}")

        elif m := PATTERN_PH.match(line):
            ph = Float32()
            ph.data = adc_to_ph(m.group(1))
            self.get_logger().info(f"pH value: {ph.data}")
            self.ph_pub.publish(ph)

        else:
            self.get_logger().warn(f"Unknown line: {line}")
        
    def ph_request(self, msg: Bool):
        if msg.data:
            message = f"<PH_REQUEST:1>\n"
            self.ser.write(message.encode("utf-8"))
            self.get_logger().info(f"Sent message: {message}")
        
    def listener_callback(self, msg:JointState):
        # This implementation sends one command over Serial
        joint_names = msg.name
        joint_values = msg.position

        # Send pH probe request DEPRICATED
        if ("ph_probe") in joint_names:
            self.get_logger().warn("THIS METHOD IS DEPRICATED: Send 'True' to 'arm/ph_request' to request a pH reading")

        else:
            # Send joint value changes
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

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


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