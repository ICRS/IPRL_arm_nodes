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
        self.gearRatioShoulder = 100.0
        self.gearRatioElbow = 50.0
        self.gearRatioWrist = 91.8
        self.gearRatioBase = 102.0
        self.upperArmLength = 325
        self.foreArmLength = 330
        self.gripperLength = 195

        # Physical parameters of two four bar linkages.
        self.fourBarL1 = 330
        self.fourBarL2 = 110
        self.fourBarL3 = 142.147
        self.fourBarL4 = 330
        self.fourBarL5 = 325
        self.fourBarL6 = 142.147
        self.fourBarL7 = 200
        self.fourBarL8 = 320

        # Angle of central joint which joins the two four bar linkages
        self.fourBarBeta = math.radians(-101.42)

        # Current angles (update via encoders)
        self.currentAngleBase = initialJointAngles[0]
        self.currentAngleShoulder = initialJointAngles[1]
        self.currentAngleElbow = initialJointAngles[2]
        self.currentAngleWrist = initialJointAngles[3]
        self.currentAngleRoll = initialJointAngles[4]

        # Desired angles
        self.desiredAngleBase = self.currentAngleBase
        self.desiredAngleShoulder = self.currentAngleShoulder
        self.desiredAngleElbow = self.currentAngleElbow
        self.desiredAngleWrist = self.currentAngleWrist
        self.desiredAngleRoll = self.currentAngleRoll
    
        # End effector variables
        self.currentGripperState = 50 #percent closed
    
    def getGripperState(self):
        return self.currentGripperState

    def getCurrentAngles(self):
        """ Outputs joint angles in degrees, order Shoulder, Elbow, Wrist, Base, Roll"""
        return [self.currentAngleShoulder, self.currentAngleElbow, self.currentAngleWrist, self.currentAngleBase, self.currentAngleRoll]
    
    def getDesiredAngles(self):
        """ Outputs desired joint angles in degrees, order Shoulder, Elbow, Wrist, Base, Roll"""
        return [self.desiredAngleShoulder, self.desiredAngleElbow, self.desiredAngleWrist, self.desiredAngleBase, self.desiredAngleRoll]

    def updateDesiredAngles(self, newStateArray):
        """newStateArray: Shoulder, Elbow, Wrist, Base, Roll"""
        self.desiredAngleShoulder = newStateArray[0]
        self.desiredAngleElbow = newStateArray[1]
        self.desiredAngleWrist = newStateArray[2]
        self.desiredAngleBase = newStateArray[3]
        self.desiredAngleRoll = newStateArray[4]

    def updateCurrentAngles(self, encoderAngles):
        self.currentAngleShoulder = encoderAngles[0]
        self.currentAngleElbow = encoderAngles[1]
        self.currentAngleWrist = encoderAngles[2]
        self.currentAngleBase = encoderAngles[3]
        self.currentAngleRoll = encoderAngles[4]

    def resetAnglesToTrue(self):
        self.updateDesiredAngles(self.getCurrentAngles())

    def calculateNewAngleWrist(self, angleWrist, angleShoulder, angleElbow):
        fourBarTheta2 = math.radians(angleWrist + 90);
        fourBarDiagonal = (self.fourBarL1**2) + (self.fourBarL2**2) - 2*self.fourBarL1*self.fourBarL2*math.cos(fourBarTheta2);
        fourBarCosTheta4 = ((self.fourBarL3**2) + (self.fourBarL4**2) - fourBarDiagonal) / (2*self.fourBarL3*self.fourBarL4);
        fourBarSinTheta4 = math.sqrt(1 - (fourBarCosTheta4**2));
        fourBarTheta4 = math.atan2(fourBarSinTheta4, fourBarCosTheta4);
        fourBarTheta3 = math.radians(180) - math.asin(self.fourBarL2*math.sin(fourBarTheta2)/math.sqrt(fourBarDiagonal)) - math.asin(self.fourBarL4*math.sin(fourBarTheta4)/math.sqrt(fourBarDiagonal));
        fourBarAlpha = math.radians(angleShoulder+angleElbow);

        fourBarTheta2Prime = math.radians(angleElbow) + fourBarTheta3 + self.fourBarBeta;
        fourBarAlphaPrime = fourBarAlpha - math.radians(angleElbow);
        fourBarDiagonalPrime = (self.fourBarL5**2) + (self.fourBarL6**2) - 2*self.fourBarL5*self.fourBarL6*math.cos(fourBarTheta2Prime);
        fourBarCosTheta4Prime = ((self.fourBarL7**2) + (self.fourBarL8**2) - fourBarDiagonalPrime) / (2*self.fourBarL7*self.fourBarL8);
        fourBarSinTheta4Prime = math.sqrt(1 - (fourBarCosTheta4Prime**2));
        fourBarTheta4Prime = math.atan2(fourBarSinTheta4Prime, fourBarCosTheta4Prime);
        fourBarTheta3Prime = math.radians(180) - math.asin(self.fourBarL6*math.sin(fourBarTheta2Prime)/math.sqrt(fourBarDiagonalPrime)) - math.asin(self.fourBarL8*math.sin(fourBarTheta4Prime)/math.sqrt(fourBarDiagonalPrime));

        fourBarInputAngle = (math.degrees(fourBarTheta3Prime) - angleShoulder);
        return fourBarInputAngle;

    def FK2D(self):
        currentPositionX = self.upperArmLength*math.cos(math.radians(self.currentAngleShoulder)) + self.foreArmLength*math.cos(math.radians(self.currentAngleShoulder + self.currentAngleElbow)) + self.gripperLength*math.cos(math.radians(self.currentAngleShoulder + self.currentAngleElbow + self.currentAngleWrist));
        currentPositionY = -self.upperArmLength*math.sin(math.radians(self.currentAngleShoulder)) - self.foreArmLength*math.sin(math.radians(self.currentAngleShoulder + self.currentAngleElbow)) - self.gripperLength*math.sin(math.radians(self.currentAngleShoulder + self.currentAngleElbow + self.currentAngleWrist));
        return (currentPositionX, currentPositionY)

    def moveLinear(self, up, distance):
        """ Moves arm forwards (x) or upwards (y) by a given distance in mm; angles in order shoulder, elbow, wrist, base """
        # Forward kinematics
        currentPositionX = self.upperArmLength*math.cos(math.radians(self.currentAngleShoulder)) + self.foreArmLength*math.cos(math.radians(self.currentAngleShoulder + self.currentAngleElbow)) + self.gripperLength*math.cos(math.radians(self.currentAngleShoulder + self.currentAngleElbow + self.currentAngleWrist));
        currentPositionY = -self.upperArmLength*math.sin(math.radians(self.currentAngleShoulder)) - self.foreArmLength*math.sin(math.radians(self.currentAngleShoulder + self.currentAngleElbow)) - self.gripperLength*math.sin(math.radians(self.currentAngleShoulder + self.currentAngleElbow + self.currentAngleWrist));

        if (up):
            currentPositionY += distance
        else:
            currentPositionX += distance
        
        # Absolute angle of the end effector: phi
        phi = math.radians(self.desiredAngleShoulder+self.desiredAngleElbow+self.desiredAngleWrist)

        # Parameters for IK. trust Otter they work
        X_ = currentPositionX - self.gripperLength*math.cos(phi);
        Y_ = currentPositionY + self.gripperLength*math.sin(phi);
        A = -2*X_*self.upperArmLength;
        B = 2*Y_*self.upperArmLength;
        C = (self.foreArmLength**2) - (self.upperArmLength**2) - (X_**2) - (Y_**2);
        ksi = math.atan2(B, A);

        # Inverse kinematics: Determine new motor angles for desired position
        newAngleShoulder = ksi + math.acos(C/math.sqrt((A**2)+(B**2)));
        newAngleElbow = math.atan2((-Y_-self.upperArmLength*math.sin(newAngleShoulder))/self.foreArmLength, (X_-self.upperArmLength*math.cos(newAngleShoulder))/self.foreArmLength) - newAngleShoulder;
        newAngleWrist = phi - newAngleShoulder - newAngleElbow;

        out_angles = [newAngleShoulder, newAngleElbow, newAngleWrist, self.desiredAngleBase]
        out_angles = [quadrant(math.degrees(item)) for item in out_angles]
        return out_angles

    def calculateDesiredAngles(self, dx, dy, dphi, dtheta, droll):
        """
        dx: +ve forwards, -ve backwards
        dy: +ve upwards, -ve downwards
        dphi: rotation about end effector
        dtheta: rotation about base
        droll: rotates end effector about its axis
        """
        changes = [dx, dy, dphi, dtheta, droll]
        #self.resetAnglesToTrue()
        
        for i in range(0, len(changes)):
            anglesToUpdate = [self.desiredAngleShoulder, self.desiredAngleElbow, self.desiredAngleWrist, self.desiredAngleBase, self.desiredAngleRoll]
            change = changes[i]
            if abs(change) > 0.5:
                if (i == 0): # dx
                    anglesToUpdate = [*self.moveLinear(0, change),self.desiredAngleRoll]
                elif (i == 1): # dy
                    anglesToUpdate = [*self.moveLinear(1, change),self.desiredAngleRoll]
                elif(i == 2): # dphi
                    #anglesToUpdate[2] = self.calculateNewAngleWrist(self.desiredAngleWrist + change, self.desiredAngleShoulder, self.desiredAngleElbow)
                    pass #TODO: FIX THIS
                elif(i==3): # dtheta
                    anglesToUpdate[3] = self.desiredAngleBase + change
                elif(i==4): # droll
                    anglesToUpdate[4] = self.desiredAngleRoll + change
            print(anglesToUpdate)
            self.updateDesiredAngles(anglesToUpdate)
    
    def getNewGripperClosure(self, dpercent):
        """dpercent: +ve more closed, -ve less closed"""
        self.currentGripperState += dpercent

if __name__ == '__main__':
    testArm = Arm()
    print("Arm created")
    print(testArm.getAnglesState())
    print(testArm.getGripperState())
    testArm.getNewAngles(0, 10, 0, 0, 0)
    testArm.getNewGripperClosure(5)
    print(testArm.getAnglesState())
    print(testArm.getGripperState())
