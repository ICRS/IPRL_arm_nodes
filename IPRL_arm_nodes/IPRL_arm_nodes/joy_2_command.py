import rclpy
from rclpy.node import Node
import arm_utilities
from sensor_msgs.msg import Joy, JointState

class Joy2Command(Node):

    def __init__(self):
        super().__init__('joy_2_command')

        # publisher
        self.joint_names = ['base','shoulder','elbow','wrist','roll','grasp']
        self.publisher_ = self.create_publisher(JointState, 'set_joint_values', 2)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        #subscriber
        self.joy_subscription = self.create_subscription(Joy, "joy", self.listener_callback, 2)
        self.joy_sample_period = self.timer_period/2
        self.ang_subscription = self.create_subscription(JointState, "read_joint_values", self.angle_callback, 2)

        # values
        self.encoder_values = [0, 0, 0, 0, 0, 0] # base shoulder elbow wrist roll grasp
        self.values_to_write = []
        self.primed_array = [False,False,False] #whether shoulder elbow wrist have been read yet
        self.primed = False

        # arm variables
        self.arm = arm_utilities.Arm([0, -90, 90, 0, 0])
        self.slow = 20
        self.fast = 50
        self.max_velocity = 5 #cm/s
        self.max_angular_speed = 5 #deg/s
        self.max_opening_speed = 3 #cm/s

    def timer_callback(self):
        if self.primed:
            new_state = self.values_to_write
            msg = JointState()
            joint_names = []
            joint_values = []
            current_state = self.encoder_values
            changed = False
            for i in range(len(new_state)):
                
                if current_state[i] != new_state[i]:
                    joint_names.append(self.joint_names[i])
                    joint_values.append(float(new_state[i]))
                    changed = True
            if changed:
                msg.name = joint_names
                msg.position = joint_values
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing: "%s"' % str(msg))
                self.values_to_write = self.encoder_values.copy()
                self.arm.updateCurrentAngles(current_state)

            # IMPORTANT: UPDATE STATE FROM ENCODERS
            # And reset values to write

    def angle_callback(self, msg:JointState):
        """Constantly checks for updated encoder values and keeps array updated"""
        joint_names = msg.name
        joint_values = msg.position
        for joint_id in range(len(joint_values)):
            self.encoder_values[joint_id] = joint_values[joint_id]

            if ((self.primed != True) and (self.primed_array[joint_id-1] == False)):
                self.primed_array[joint_id-1] = True
                if self.primed_array==[True,True,True]:
                    self.arm.updateCurrentAngles(self.encoder_values)
                    self.values_to_write = self.encoder_values.copy()
                    self.primed = True
                    self.get_logger().info("Arm is primed, ready to move")
                    self.get_logger().info("Initial values: %s" % str(self.encoder_values))

    def map_buttons(self, button_array):
        """ Maps buttons on controller to False if unpressed, True if pressed"""
        button_dict = {}

        button_dict["SLOW"] = bool(button_array[4])
        button_dict["FAST"] = bool(button_array[5])

        return button_dict

    def map_axes(self, axes_array):
        """ Maps open/close to True if pressed; maps base/y/z/roll/endpoint angle to value between -1 and 1"""
        axes_dict = {}

        axes_dict["OPEN"] = bool(axes_array[5]==-1)
        axes_dict["CLOSE"] = bool(axes_array[2]==-1)

        axes_dict["BASE"] = float(axes_array[0])
        axes_dict["Y"] = float(axes_array[1])
        axes_dict["Z"] = float(axes_array[7])

        axes_dict["ROLL"] = float(axes_array[3])
        axes_dict["ENDPOINT_ANGLE"] = float(axes_array[4])

        return axes_dict

    def listener_callback(self, msg):
        if self.primed:
            
            buttons_dict = self.map_buttons(msg.buttons)
            axes_dict = self.map_axes(msg.axes)
            # dead man's switch on all but wrist roll and grasp; affected by SLOW, FAST
            speed = 0
            if buttons_dict["SLOW"] == 1:
                speed = self.slow
            elif buttons_dict["FAST"] == 1:
                speed = self.fast
            if speed != 0:
                # Check for base rotation; affected by BASE
                # Value of new_state[0] is change in base angle about horizontal
                if axes_dict["BASE"] != 0:
                    self.values_to_write[0] = self.values_to_write[0] + axes_dict["BASE"]*speed * self.joy_sample_period
                # Find new IK angles; affected by Z, Y, ENDPOINT_ANGLE
                # Value of new_state[1:2] is absolute angle to move to
                if ((axes_dict["Z"] != 0) | (axes_dict["Y"] != 0) | (axes_dict["ENDPOINT_ANGLE"] != 0)):
                    new_angles = self.arm.IK2D(axes_dict["Y"]*speed*self.max_velocity*self.joy_sample_period, axes_dict["Z"]*speed*self.max_velocity*self.joy_sample_period, axes_dict["ENDPOINT_ANGLE"]*speed*self.max_angular_speed*self.joy_sample_period)
                    self.values_to_write[1:4] = new_angles
            # Wrist roll; affected by ROLL
            # Value of new_state[4] is number of seconds to roll wrist, sense depending on sign
            self.values_to_write[4] = self.values_to_write[4] + self.joy_sample_period*axes_dict["ROLL"]

            # Grasp; affected by OPEN, CLOSE
            # Value of new_state[5] is speed gripper should open/close
            self.values_to_write[5] = 0
            if axes_dict["OPEN"]:
                self.values_to_write[5] = self.max_opening_speed
            if axes_dict["CLOSE"]:
                self.values_to_write[5] = -1*self.max_opening_speed
                if axes_dict["OPEN"]:
                    # If both triggers pulled, stop moving
                    self.values_to_write[5] = 0


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