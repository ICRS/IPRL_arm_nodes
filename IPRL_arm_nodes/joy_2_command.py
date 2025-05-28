import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

class Joy2Command(Node):

    def __init__(self):
        super().__init__('joy_2_command')

        # publisher
        self.publisher_ = self.create_publisher(Float64MultiArray, 'commands', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        #subscriber
        self.subscription = self.create_subscription(String, "keystroke", self.listener_callback, 10)

        # values
        self.state_update = [0,0,0,0,0] #[dx,dy,dphi,dtheta,droll]
        self.speed = 5

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [float(item) for item in self.state_update]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

        # IMPORTANT: RESET STATE UPDATE
        self.state_update = [0,0,0,0,0]

    def listener_callback(self, msg):
        key = msg.data
        keyboard_mapping = [('w','s'),('i','k'),('e','n'),('d','a'),('l','j')] #[dx,dy,dphi,dtheta,droll]
        for i in range(0, len(keyboard_mapping)):
            pair = keyboard_mapping[i]
            if(key == pair[0]):
                self.state_update[i] = self.state_update[i] + self.speed
            elif(key == pair[1]):
                self.state_update[i]  = self.state_update[i] - self.speed

def main(args=None):
    rclpy.init(args=args)

    joy_2_command = Joy2Command()

    rclpy.spin(joy_2_command)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joy_2_command.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()