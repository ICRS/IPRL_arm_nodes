import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RawSerialConsole(Node):
    def __init__(self) -> None:
        super().__init__("raw_serial_console")

        self.declare_parameter("tx_topic", "raw_tx")
        self.declare_parameter("prompt", "> ")

        self._tx_topic = str(self.get_parameter("tx_topic").value)
        self._prompt = str(self.get_parameter("prompt").value)

        self._pub = self.create_publisher(String, self._tx_topic, 10)

        self._stop_event = threading.Event()
        self._input_thread = None

        if sys.stdin.isatty():
            self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
            self._input_thread.start()

            self.get_logger().info(
                "Type raw commands like <DES_VAL:1,45> and press Enter."
            )
            self.get_logger().info("Type 'exit' or 'quit' to stop.")
        else:
            self.get_logger().warn(
                "stdin is not a TTY; console input disabled. "
                "Run 'ros2 run my_arm_serial_bridge serial_console' in a terminal."
            )

    def _input_loop(self) -> None:
        while rclpy.ok() and not self._stop_event.is_set():
            if self._prompt:
                sys.stdout.write(self._prompt)
                sys.stdout.flush()
            line = sys.stdin.readline()
            if not line:
                break
            line = line.strip()
            if not line:
                continue
            if line.lower() in ("exit", "quit"):
                rclpy.shutdown()
                break

            msg = String()
            msg.data = line
            self._pub.publish(msg)

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._input_thread is not None and self._input_thread.is_alive():
            self._input_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RawSerialConsole()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
