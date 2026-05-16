import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
from serial import SerialException


class RawSerialBridge(Node):
    def __init__(self) -> None:
        super().__init__("raw_serial_bridge")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("read_timeout_ms", 20)
        self.declare_parameter("write_timeout_ms", 200)
        self.declare_parameter("flush_on_write", False)

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        read_timeout_ms = int(self.get_parameter("read_timeout_ms").value)
        write_timeout_ms = int(self.get_parameter("write_timeout_ms").value)
        self._flush_on_write = bool(self.get_parameter("flush_on_write").value)

        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=read_timeout_ms / 1000.0,
                write_timeout=write_timeout_ms / 1000.0,
            )
        except SerialException as exc:
            self.get_logger().error(f"Failed to open serial port {port}: {exc}")
            raise

        self.get_logger().info(f"Opened {port} at {baud} baud")

        self._rx_pub = self.create_publisher(String, "raw_rx", 10)
        self.create_subscription(String, "raw_tx", self._on_tx, 10)

        self._tx_lock = threading.Lock()

        self._stop_event = threading.Event()
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

    def _on_tx(self, msg: String) -> None:
        line = msg.data
        if not line:
            return
        if not line.endswith("\n"):
            line += "\n"
        try:
            with self._tx_lock:
                self._serial.write(line.encode("utf-8"))
                if self._flush_on_write:
                    self._serial.flush()
        except SerialException as exc:
            self.get_logger().error(f"Serial write failed: {exc}")

    def _read_loop(self) -> None:
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                line_bytes = self._serial.readline()
            except SerialException as exc:
                self.get_logger().error(f"Serial read failed: {exc}")
                break

            if not line_bytes:
                continue

            line = line_bytes.decode("utf-8", errors="replace").rstrip("\r\n")
            if not line:
                continue

            msg = String()
            msg.data = line
            self._rx_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._read_thread.is_alive():
            self._read_thread.join(timeout=1.0)
        if self._serial.is_open:
            self._serial.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RawSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
