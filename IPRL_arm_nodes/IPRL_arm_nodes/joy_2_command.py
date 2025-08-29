import rclpy
from rclpy.node import Node
import arm_utilities
from sensor_msgs.msg import Joy, JointState

class Joy2Command(Node):

    def __init__(self):
        super().__init__('joy_2_command')

        # publisher
        self.joint_names = ["base","shoulder","elbow","wrist","roll","grasp"]
        self.publisher_ = self.create_publisher(JointState, "set_joint_values", 2)
        self.timer_period = 0.2  # seconds
        self.timer = self.create_timer(self.timer_period, self.publisher_callback)
        
        # subscriber
        self.joy_subscription = self.create_subscription(Joy, "/arm/joy", self.controller_callback, 2)
        self.joy_sample_period = 1/15 # Depends on controller frequency
        self.ang_subscription = self.create_subscription(JointState, "read_joint_values", self.angle_callback, 2)

        # values
        self.encoder_values = [0, 0, 0, 0, 0, 0] # base shoulder elbow wrist roll grasp
        self.value_delta = {}
        self.prev_value_delta = {}
        self.end_effector_incs = [0,0,0]
        self.get_ph = False
        self.primed_array = [False,False,False] #whether shoulder elbow wrist have been read yet
        self.primed = False

        # arm variables
        self.arm = arm_utilities.Arm([0, -90, 90, 0, 0])
        self.slow = 0.5
        self.fast = 1
        self.max_velocity = 50 #mm/s
        self.max_angular_speed = 15 #deg/s
        self.max_opening_speed = 3 #cm/s
        self.movement_threshold = 0.1 #deg per call
        self.roll_speed = 10 #deg/s

        self.get_logger().info("Started joy_2_command node, waiting to read initial arm position")

    def publisher_callback(self):
        if self.primed:
            # Update arm state
            current_state = self.encoder_values
            self.arm.updateCurrentAngles(current_state)

            # Perform arm IK
            if self.end_effector_incs != [0,0,0]:
                # Find new shoulder elbow wrist angles
                new_SEW = self.arm.IK2D(self.end_effector_incs[0], self.end_effector_incs[1], self.end_effector_incs[2])
                for i in range (0,len(new_SEW)):
                    name = self.joint_names[i+1]
                    self.value_delta[name] = new_SEW[i]-current_state[i+1]

            # Init joint message
            msg = JointState()
            joint_names = []
            joint_values = []

            # Check if joint should be changed
            for joint_name in self.joint_names:
                # If joint actively being changed
                if joint_name in self.value_delta:
                    changed_joint = joint_name
                    delta = self.value_delta[changed_joint]
                    joint_id = self.joint_names.index(changed_joint)
                    if ((changed_joint=="base") or (changed_joint=="grasp")): #base and grasp are special cases, already differential
                        joint_names.append(changed_joint)
                        joint_values.append(delta)
                    elif (((abs(delta) < self.movement_threshold) and (joint_id<4)) or (changed_joint=="roll" and delta==0)):
                        pass # angle movement under threshold or no rotation time
                    else: #differential
                        joint_names.append(changed_joint)
                        joint_values.append(current_state[joint_id] + delta)
                # If joint not actively changed
                elif joint_name in self.prev_value_delta:
                    if ((joint_name=="base") or (joint_name=="grasp") or (joint_name=="roll")):
                        joint_names.append(joint_name)
                        joint_values.append(0)
                    else:
                        joint_names.append(joint_name)
                        joint_values.append(self.encoder_values[self.joint_names.index(joint_name)])

            if joint_names:
                msg.name = joint_names
                msg.position = joint_values
                self.publisher_.publish(msg)
                self.get_logger().info('Setting joints "%s" to "%s"' % (str(msg.name), str(msg.position)))

            # Special case: pH reading
            if self.get_ph:
                # Separate message. TODO: Make new topic
                ph_msg = JointState()
                ph_msg.name = ["ph_probe"]
                self.publisher_.publish(ph_msg)
                self.get_logger().info('Requested pH value')
                self.get_ph = False

            # IMPORTANT: Reset value changes and end effector increments
            self.prev_value_delta = self.value_delta.copy()
            self.value_delta = {}
            self.end_effector_incs = [0,0,0]

    def angle_callback(self, msg:JointState):
        """Constantly checks for updated encoder values and keeps array updated"""
        joint_names = msg.name
        joint_values = msg.position

        #self.get_logger().info("Received message: %s" % str(msg))
        for joint_name in joint_names:
            joint_id = self.joint_names.index(joint_name)
            self.encoder_values[joint_id] = joint_values[joint_names.index(joint_name)]

            if ((joint_name != "base") and (self.primed != True) and (self.primed_array[joint_id-1] == False)):
                self.primed_array[joint_id-1] = True
                if self.primed_array==[True,True,True]:
                    self.arm.updateCurrentAngles(self.encoder_values)
                    self.primed = True
                    self.get_logger().info("Arm is primed, ready to move")
                    self.get_logger().info("Initial values: %s" % str(self.arm.getCurrentAngles()))

                    # Lock arm into original state on startup
                    init_msg = JointState()
                    init_msg.name = ["base", "shoulder", "elbow", "wrist"]
                    init_msg.position = self.encoder_values[0:4]
                    self.publisher_.publish(init_msg)

    def map_buttons(self, button_array):
        """ Maps buttons on controller to False if unpressed, True if pressed"""
        button_dict = {}

        button_dict["SLOW"] = bool(button_array[4])
        button_dict["FAST"] = bool(button_array[5])

        button_dict["PH"] = bool(button_array[2])

        return button_dict

    def map_axes(self, axes_array):
        """ Maps open/close to True if pressed; maps base/y/z/roll/endpoint angle to value between -1 and 1"""
        axes_dict = {}

        axes_dict["OPEN"] = bool(axes_array[5]==-1)
        axes_dict["CLOSE"] = bool(axes_array[2]==-1)

        axes_dict["BASE"] = float(axes_array[0])
        axes_dict["Y"] = float(axes_array[1])
        axes_dict["Z"] = float(axes_array[7])

        axes_dict["ROLL"] = -1*float(axes_array[3])
        axes_dict["ENDPOINT_ANGLE"] = float(axes_array[4])

        return axes_dict

    def controller_callback(self, msg):
        if self.primed:
            
            buttons_dict = self.map_buttons(msg.buttons)
            axes_dict = self.map_axes(msg.axes)
            # dead man's switch on all but wrist roll and grasp; affected by SLOW, FAST
            speed = 0
            if buttons_dict["SLOW"]:
                speed = self.slow
            elif buttons_dict["FAST"]:
                speed = self.fast
            if speed:
                # Check for base rotation; affected by BASE
                # Value of new_state[0] is change in base angle about horizontal
                if axes_dict["BASE"] != 0:
                    self.value_delta["base"] = axes_dict["BASE"]*speed*self.max_angular_speed
                
                # Find new IK angles; affected by Z, Y, ENDPOINT_ANGLE
                # Value of new_state[1:2] is absolute angle to move to
                if ((axes_dict["Z"] != 0) | (axes_dict["Y"] != 0) | (axes_dict["ENDPOINT_ANGLE"] != 0)):
                    self.end_effector_incs = [axes_dict["Y"]*speed*self.max_velocity, axes_dict["Z"]*speed*self.max_velocity, axes_dict["ENDPOINT_ANGLE"]*speed*self.max_angular_speed]

                # Wrist roll; affected by ROLL
                # Value of new_state[4] is speed to roll wrist, sense depending on sign
                if (axes_dict["ROLL"]): 
                    self.value_delta["roll"] = axes_dict["ROLL"]*speed*self.roll_speed
            # Grasp; affected by OPEN, CLOSE
            # Value of new_state[5] is speed gripper should open/close
            if (("grasp" in self.prev_value_delta) and (self.prev_value_delta["grasp"] != 0)):
                # Defaults to 0
                self.value_delta["grasp"] = 0
            if axes_dict["OPEN"]:
                self.value_delta["grasp"] = self.max_opening_speed
            if axes_dict["CLOSE"]:
                self.value_delta["grasp"] = -1*self.max_opening_speed
                if axes_dict["OPEN"]:
                    # If both triggers pulled, stop moving
                    self.value_delta["grasp"] = 0
            
            # pH Probe
            if buttons_dict["PH"]:
                self.get_ph = True


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