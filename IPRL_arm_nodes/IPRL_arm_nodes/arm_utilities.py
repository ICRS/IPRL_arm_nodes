import math

def quadrant(angle):
    if (angle > 180):
        return (angle-360)
    elif (angle <-180):
        return (angle+360)
    else:
        return angle

class Arm():
    def __init__(self, initialJointAngles):
        # PARAMETERS
        # Physical parameters of arm.
        self.upperArmLength = 325
        self.foreArmLength = 330
        self.gripperLength = 290

        # Current angles (update via encoders)
        self.currentAngleBase = initialJointAngles[0]
        self.currentAngleShoulder = initialJointAngles[1]
        self.currentAngleElbow = initialJointAngles[2]
        self.currentAngleWrist = initialJointAngles[3]
        self.currentAngleRoll = initialJointAngles[4]

    def getCurrentAngles(self):
        """ Outputs joint angles in degrees, order Shoulder, Elbow, Wrist, Base, Roll"""
        return [self.currentAngleBase, self.currentAngleShoulder, self.currentAngleElbow, self.currentAngleWrist, self.currentAngleRoll]
    
    def getDesiredAngles(self):
        """ Outputs desired joint angles in degrees, order Shoulder, Elbow, Wrist, Base, Roll"""
        return [self.desiredAngleBase, self.desiredAngleShoulder, self.desiredAngleElbow, self.desiredAngleWrist, self.desiredAngleRoll]

    def updateCurrentAngles(self, receivedAngles):
        self.currentAngleBase = receivedAngles[0]
        self.currentAngleShoulder = receivedAngles[1]
        self.currentAngleElbow = receivedAngles[2]
        self.currentAngleWrist = receivedAngles[3]
        self.currentAngleRoll = receivedAngles[4]

    def resetAnglesToTrue(self):
        self.updateDesiredAngles(self.getCurrentAngles())

    def FK2D(self):
        currentPositionX = self.upperArmLength*math.cos(math.radians(self.currentAngleShoulder)) + self.foreArmLength*math.cos(math.radians(self.currentAngleShoulder + self.currentAngleElbow)) + self.gripperLength*math.cos(math.radians(self.currentAngleShoulder + self.currentAngleElbow + self.currentAngleWrist));
        currentPositionY = -self.upperArmLength*math.sin(math.radians(self.currentAngleShoulder)) - self.foreArmLength*math.sin(math.radians(self.currentAngleShoulder + self.currentAngleElbow)) - self.gripperLength*math.sin(math.radians(self.currentAngleShoulder + self.currentAngleElbow + self.currentAngleWrist));
        return (currentPositionX, currentPositionY)

    def IK2D(self, dx, dy, dphi):
        """ Moves arm forwards (x) or upwards (y) by a given distance in mm; angles in order shoulder, elbow, wrist """
        # Forward kinematics
        currentPosition = self.FK2D()

        desiredPositionX = currentPosition[0] + dx
        desiredPositionY = currentPosition[1] + dy

        # Prevent out of bounds dying
        if math.sqrt(desiredPositionX**2 + desiredPositionY**2) > (self.upperArmLength+self.foreArmLength+self.gripperLength-15):
            return [self.currentAngleShoulder, self.currentAngleElbow, self.currentAngleWrist]
        
        # Absolute angle of the end effector: phi
        phi = math.radians(self.currentAngleShoulder+self.currentAngleElbow+self.currentAngleWrist) + math.radians(dphi)

        # Parameters for IK
        X_ = desiredPositionX - self.gripperLength*math.cos(phi); # x'=x-L_4C_phi
        Y_ = desiredPositionY + self.gripperLength*math.sin(phi); # y'=y+L_4S_phi
        A = -2*X_*self.upperArmLength; # A=-2x'L_2
        B = 2*Y_*self.upperArmLength; # B=2y'L_2
        C = (self.foreArmLength**2) - (self.upperArmLength**2) - (X_**2) - (Y_**2); #C=L_3^2 - L_2^2 - x'^2 - y'^2
        ksi = math.atan2(B, A); # xi = atan2(B, A)

        # Inverse kinematics: Determine new motor angles for desired position
        newAngleShoulder = ksi + math.acos(C/math.sqrt((A**2)+(B**2))); #theta_2
        newAngleElbow = math.atan2((-Y_-self.upperArmLength*math.sin(newAngleShoulder))/self.foreArmLength, (X_-self.upperArmLength*math.cos(newAngleShoulder))/self.foreArmLength) - newAngleShoulder; #theta_3
        newAngleWrist = phi - newAngleShoulder - newAngleElbow; #theta_4

        out_angles = [newAngleShoulder, newAngleElbow, newAngleWrist]
        out_angles = [quadrant(math.degrees(item)) for item in out_angles]
        return out_angles

if __name__ == '__main__':
    testArm = Arm([0, -80, 128, 44, 0])
    print("Arm created")

    print(testArm.FK2D())
    print(testArm.IK2D(0,0,0))

    new_angles = testArm.IK2D(-50,0,0)
    print(new_angles)

    
