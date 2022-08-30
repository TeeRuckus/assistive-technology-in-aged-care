import rospy
import miro2 as miro
import os
import numpy as np
import math
import time
import sys


from fallen_analyser.msg import coords
from std_msgs.msg import Float32MultiArray, UInt16MultiArray, UInt32MultiArray
from std_msgs.msg import UInt8, UInt16, UInt32, Bool
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import JointState, Imu
from Errors import *
from  miro2.lib.RobotInterface import RobotInterface
#from enum import Enum

NODE_NAME = "MiRoKinematics"
BUFFER_MAX = 5
#TODO: you will need to make your own sound directories which will have your own sounds
HELP_SIGNAL = ""

#TODO: when you will initialise the package, you'll need to make sure that MiRo's eyelids are going to be open
#TODO: When the head is being moved, you want to ignore the incoming coordinates from the pose_fallen node
#TODO: you'll need to transform the use of lists into using arrays for faster access time


class MiRoKinematics:

    def __init__(self):
        self.__verbose = False
        self.__headMoving = False
        #if resident has already fallen, ignore messages from resident/fallen/
        #topic, and initiate the MiRo fallen interaction
        self.__fallenState = False
        self.__kinHead = JointState()
        self.__kinHead.position = [0.0, math.radians(34.0), 0.0, 0.0]
        self.__startTime = 0.0
        #self.__kinHead.name = ["TILT", "LIFT", "YAW", "PITCH"]
        self.__facePos = (0,0)
        self.__ledTimer = 0
        self.__sensorInfo = None
        self.__miroStopped = False
        self.__LEDToggle = True
        self.__residentOkay = False
        rospy.init_node(NODE_NAME, anonymous=True)

        topicBaseName = "/" + os.getenv("MIRO_ROBOT_NAME")

        #SUBSCRIBERS
        #subscribing to sonar and cliff sensors of MiRo
        self.sensorPackage = rospy.Subscriber(topicBaseName + "/sensors/package",
            miro.msg.sensors_package, self.callBackSensorPackage, queue_size=1,
            tcp_nodelay=True)

        rospy.Subscriber("resident/fallen/", Bool, self.callBackHasFallen)
        rospy.Subscriber("resident/coords/", coords, self.callbackCoords)

        #PUBLISHERS
        self.pubIllum = rospy.Publisher(topicBaseName + "/control/illum",
                UInt32MultiArray, queue_size=0)

        topic = topicBaseName + "/control/kinematic_joints"
        self.__pubKin = rospy.Publisher(topic, JointState, queue_size=0)

        topic = topicBaseName + "/control/cmd_vel"
        self.__pubWheels = rospy.Publisher(topic, TwistStamped, queue_size=0)

        self.__pubWarningSignal = rospy.Publisher("resident/warningSignal/",
                Bool, queue_size=0)



        #TODO: you will add some code here to make sure that the LED lights are switched off

    @property
    def verbose(self):
        return self.__verbose

    @verbose.setter
    def verbose(self, newVerbose):
        self.__verbose = self.__validateVerbose(newVerbose)

    def moveHeadCords(self):
        """
        ASSERTION: Moves head to given xCord and yCord, if points are wthin
        MiRo's workspace
        """
        timeHeadDelay = 0

        while not rospy.core.is_shutdown():
            if self.__verbose:
                rospy.loginfo_once("Moving head given subscribed coordinates")

            timeHeadDelay = time.time() - self.__startTime


            print("x: ", self.__facePos[0])
            self.__kinHead.position[miro.constants.JOINT_YAW] = \
                self.moveHead2XCoord(self.__facePos[0])

            self.checkHeadWorkspaceYaw()
            self.checkHeadWorkSpaceLift()
            self.checkHeadWorkSpacePitch()
            print("yaw: ", self.__kinHead.position[miro.constants.JOINT_YAW])
            self.__pubKin.publish(self.__kinHead)



            #don't want to change the position wile the head is moving 
            #if abs(self.__facePos[0]) > 100 and not self.__headMoving:
            """
            if abs(self.__facePos[0]) > 100 and not self.__headMoving == True:
                print("x: ", self.__facePos[0])
                self.__kinHead.position[miro.constants.JOINT_YAW] += \
                    self.moveHead2XCoord(self.__facePos[0])

                print("yaw: ", self.__kinHead.position[miro.constants.JOINT_YAW])
                self.__headMoving = True
                self.__startTime = time.time()



            if abs(self.__facePos[1]) > 30 and not self.__headMoving == True:
                self.__kinHead.position[miro.constants.JOINT_LIFT] += \
                    self.moveHead2YCoord(self.__facePos[1])

                self.__headMoving = True
                self.__startTime = time.time()

            if not self.__headMoving == True:
                self.__headMoving = True
                self.__startTime = time.time()

            if self.__headMoving == True:
                pass


            if timeHeadDelay > 1:
                self.__headMoving = False
                #resetting the delay back to 0
                timeHeadDelay = 0

            if self.__verbose:
                rospy.loginfo("LIFT: %.2f , PITCH: %.2f , YAW: %.2f" %
                        (self.__kinHead.position[miro.constants.JOINT_TILT],
                            self.__kinHead.position[miro.constants.JOINT_PITCH],
                            self.__kinHead.position[miro.constants.JOINT_YAW]))
            """



            time.sleep(0.02)

    def respondFallen(self):
        """
        PURPOSE:
        """

        sleepTime = 0.02
        frequency = 1 / 0.02
        helpTimerStart = 0.0
        elapsedTime = 0.0

        time.sleep(0.02)

        while not rospy.core.is_shutdown():
            if self.__fallenState and not self.__miroStopped:
                rospy.loginfo_once("Getting to fallen resident...")
                #activating the wheels to drive forward
                msgWheels = TwistStamped()
                #this is the maximum forward speed of MiRo
                msgWheels.twist.linear.x = miro.constants.WHEEL_MAX_SPEED_M_PER_S
                msgWheels.twist.angular.z = 0.0
                self.__pubWheels.publish(msgWheels)

                #checking if the sonar sensor is close to something
            #if they is some sensor information, we want to determine what to do
                if not self.__sensorInfo is None:

                    currSensorInfo = self.__sensorInfo
                    #clearing sensor information not to clutter memory and 
                    #get convoluted readings
                    self.__sensorInfo = None
                    sonarReading = currSensorInfo.sonar.range

                    #if sonarReading <= 0.4:
                    if sonarReading <= 0.3:
                        self.__miroStopped = True
                        rospy.loginfo("STOP NIGGA")
                        helpTimerStart = time.time()

                        #starting the timer

            #if miro has stopped and the person has fallen, want to approach person
            if self.__fallenState and self.__miroStopped:
                #you want to turn on the LED lights to a particular color
                rospy.loginfo_once("Social Interaction MiRO")

                #checking if the resident is okay
                self.residentOkayHaptic()


                #only count the time if the resident is not okay
                if not(self.__residentOkay):
                    elapsedHelpTime = time.time() - helpTimerStart

                warningSignal = Bool()
                #waiting for 10 seconds
                if elapsedHelpTime >= 10 and not(self.__residentOkay):
                    rospy.loginfo_once("The resident is NOT okay please help ...")
                    red = np.array([255, 0, 0])
                    self.activateSOSLED(sleepTime, frequency, red, 255)
                    self.getAttention()
                    warningSignal.data = True
                    self.__pubWarningSignal.publish(warningSignal)

                if self.__residentOkay:
                    #re-setting the timers, and variables used 
                    elapsedTime = 0.0
                    helpTimerStart = 0.0
                    rospy.loginfo_once("Resident is okay now")
                    warningSignal.data = False
                    self.__pubWarningSignal.publish(warningSignal)
                    self.turnLEDOff()

                    if (self.residentOkayHaptic()):
                        rospy.loginfo("I am switching off now :)")
                        sys.exit()


            time.sleep(sleepTime)

    def moveHead2XCoord(self, dx):
        """
        ASSERTION:
        """
        retValue = 0

        """
        if self.__facePos[0] > 200:
            retValue =  math.radians((dx)*-0.1)
        elif self.__facePos[0] < 100:
            #retValue = math.radians((dx)*0.1)
            retValue = math.radians(10.0)
        """
        retValue =  math.radians((dx)*-0.1)

        #the middle of 
        """
        self.__startTime = time.time()
        self.__headMoving = True
        """
        return retValue
        #return math.radians((dx)*0.05)

    def rotateBodyRight(self):
        """
        ASSERTION: Rotates Miro to the right 1 degree at a time
        """

        if self.__verbose:
            rospy.loginfo("rotating Miro's body right")

        msgWheels = TwistStamped()
        msgWheels.twist.angular.z = 1.0
        self.__pubWheels.publish(msgWheels)
        #giving time for the command to fully execute
        time.sleep(0.01)

        return 1

    def rotateBodyLeft(self):
        """
        ASSERTION: Rotates Miro to the left 1 degree at a time
        """

        if self.__verbose:
            rospy.loginfo("rotating MiRo's body left")

        msgWheels = TwistStamped()
        msgWheels.twist.angular.z = -1.0
        self.__pubWheels.publish(msgWheels)

        time.sleep(0.01)

        return 1

    def getAttention(self):
        """
        PURPOSE:
        """

        #setting it to spin in clockwise direction
        spin = -1.0
        msgWheels = TwistStamped()
        #msgWheels.twist.linear.x = miro.constants.WHEEL_MAX_SPEED_M_PER_S*0.5
        msgWheels.twist.linear.x = 0.0
        #TODO: you can add timers on this variable to make it rotate for a period of time
        v = 1.0
        msgWheels.twist.angular.z = v * 6.2832 * spin
        self.__pubWheels.publish(msgWheels)

    def wagTail(self):
        """
        ASSERTION: Wags MiRo's tail for a set time to communicate happiness.
        """
        #TODO: come back and finish this  off

        if self.__verbose:
            rospy.loginfo("Wagging MiRo's tail")

    def residentOkayHaptic(self):
        """
        ASSERTION: Will return true if the body of Miro is touched
        """

        touched = False
        if not self.__sensorInfo is None:
            headSensor =  self.__sensorInfo.touch_head.data
            #accounting for the sensors which are already pressed
            bodySensor = self.__sensorInfo.touch_body.data - 14208

            if self.__verbose:
                rospy.loginfo("headSensor: %s bodySensor: %s " % (headSensor, 
                    bodySensor))

            if (headSensor > 0) | (bodySensor > 0):
                self.__residentOkay = True
                touched = True
                rospy.loginfo("You touched me ...")

        return touched

        #grabbing the head sensor data

    def turnLEDOn(self, rgb, bright):
        """
        PURPOSE
        """

        frontLeft, midLeft, rearLeft, frontRight, midRight, rearRight = range(6)
        msgIllum = UInt32MultiArray()
        msgIllum.data = [0, 0, 0, 0, 0, 0]

        value = self.__generateIllum(rgb, bright)

        #turning front section of lights on
        msgIllum.data[frontLeft] = value
        msgIllum.data[frontRight] = value

        #turning midsection of lights on
        msgIllum.data[midLeft] = value
        msgIllum.data[midRight] = value

        #turning rear section of lights on
        msgIllum.data[rearLeft] = value
        msgIllum.data[rearRight] = value

        self.pubIllum.publish(msgIllum)

    def activateSOSLED(self, sleep, frequency, color, bright):
        """
        ASSERTION: MiRO will continuously turn on the LED on and off with
        intervals of 1 second given the specified color and brightness
        """
        self.__ledTimer += 1


        if ((self.__ledTimer % frequency) == 0) and self.__LEDToggle:
            self.turnLEDOn(np.array([255,0,0]), 255)
            self.__LEDToggle = False
        else:
            self.turnLEDOff()
            self.__LEDToggle = True

    def turnLEDOff(self):
        """
        ASSERTION: Turns all sections of MiRO
        """
        msgIllum = UInt32MultiArray()
        msgIllum.data = [0, 0, 0, 0, 0, 0]
        self.pubIllum.publish(msgIllum)

    #TODO: I honestly don't know what this function is actually going to do
    def wiggle(self, v, n, m):
        """
        ASSERTION: Wiggles MiRo's tail within the tail's work space
        """
        #TODO: come back and finish this  off
        v = v + float(n) / float(m)
        if v > 2.0:
            v -= 2.0
        elif v > 1.0:
            v = 2.0 - v
        return v

        #stop node from being excited until node has being stopped

    def moveHead2YCoord(self, dy):
        """
        ASSERTION:
        """
        self.__startTime = time.time()
        self.__headMoving = True
        #THIS IS GOING TO BE WITH LIFT
        return math.radians((abs(dy)-20)*np.sign(dy)*0.3)

    def checkHeadWorkSpaceLift(self):
        """
        ASSERTION: Will stop MiRo's head from moving past it's workspace in the
        lift direction.
        """

        workspaceOkay = True

        if self.__kinHead.position[miro.constants.JOINT_LIFT] >= miro.constants.LIFT_RAD_MAX:
            self.__kinHead.position[miro.constants.JOINT_LIFT] = \
            miro.constants.LIFT_RAD_MAX - BUFFER_MAX
            workspaceOkay = False
            self.__headMoving = True

            if self.__verbose:
                rospy.loginfo("Miro has reached lift max")

        if self.__kinHead.position[miro.constants.JOINT_LIFT] <= miro.constants.LIFT_RAD_MIN:
            self.__kinHead.position[miro.constants.JOINT_TILT] = \
            miro.constants.LIFT_RAD_MIN + BUFFER_MAX
            workspaceOkay = False
            self.__headMoving = True

            if self.__verbose:
                rospy.loginfo("Miro has reached lift minimum")


        return workspaceOkay

    def checkHeadWorkSpacePitch(self):
        """
        ASSERTION: Will stop MiRo's head from moving past it's workspace in the
        pitch direction.
        """

        workspaceOkay = True

        if self.__kinHead.position[miro.constants.JOINT_PITCH] >= miro.constants.PITCH_RAD_MAX:
            self.__kinHead.position[miro.constants.JOINT_PITCH] = \
            miro.constants.PITCH_RAD_MAX - BUFFER_MAX

            workspaceOkay = False

            if self.__verbose:
                rospy.loginfo("Miro has reached pitch's max")

        if self.__kinHead.position[miro.constants.JOINT_PITCH] <= miro.constants.PITCH_RAD_MIN:
            self.__kinHead.position[miro.constants.JOINT_PITCH] = \
            miro.constants.PITCH_RAD_MIN + BUFFER_MAX

            workspaceOkay = False

            if self.__verbose:
                rospy.loginfo("Miro has reached pitch's min")

        return workspaceOkay

    def checkHeadWorkspaceYaw(self):
        """
        ASSERTION: Will stop MiRo's head from past it's workspace in the yaw
        direction.
        """
        workspaceOkay = True

        if self.__kinHead.position[miro.constants.JOINT_YAW] >= miro.constants.YAW_RAD_MAX:
            self.__kinHead.position[miro.constants.JOINT_YAW] = \
            miro.constants.YAW_RAD_MAX - BUFFER_MAX

            workspaceOkay = False
            self.__headMoving = True
            if self.__verbose:
                rospy.loginfo("Miro has reached yaw's max")

        if self.__kinHead.position[miro.constants.JOINT_YAW] <= miro.constants.YAW_RAD_MIN:
            self.__kinHead.position[miro.constants.JOINT_YAW] = \
                miro.constants.YAW_RAD_MIN + BUFFER_MAX


            workspaceOkay = False
            self.__headMoving = True
            if self.__verbose:
                rospy.loginfo("Miro has reached yaw's minimum")

        return workspaceOkay

    def callbackCoords(self, data):
        """
        ASSERTION: This will unpack subscribed data from resident/coords/ topic and
        assign data to the facePos class field
        """
        #add another point to the path of residents head
        self.__facePos = (int(data.xCord.data), int(data.yCord.data))

        if self.__verbose:
            rospy.loginfo("received Coordinates: (%.3f, %.3f)" %
                    (data.xCord.data, data.yCord.data))

    def callBackHasFallen(self, data):
        """
        ASSERTION: This function will unpack subscribed data from resident/fallen/
        topic and assign the fallenState class field to True
        """

        #if the fallen state has already being activated and hasn't be cleared,
        #we don't want to accept any new incoming data
        if not self.__fallenState:
            self.__fallenState = bool(data.data)

        if self.__verbose:
            rospy.loginfo("Resident hasn't fallen: %s " % data.data)

    def callBackSensorPackage(self, msg):
        self.__sensorInfo = msg

    def callbackKin(self, msg):
        """
        """
        #TODO: you'll need to have some protection, so that if it's not called it will do nothing
        self.__kinHead = msg.position

    #TODO: come back and expand on this idea and see what can come out of it
    def cleanUp(self):
        self.turnLEDOff()
        print("you have shut me down mate")

    def __validateVerbose(self, inVerbose):
        if type(inVerbose) is not(bool):
            MiRoError("Verbose muse be a boolean either true or false")

        return inVerbose

    def __generateIllum(self, rgb, bright):
        """
        ASSERTION: Generates MiRo specific code to produce color given rgb import
        with specified brightness by the bright import.
        """
        red = rgb[0]
        green = rgb[1]
        blue = rgb[2]
        return (int(bright) << 24) | (red << 16) | (green << 8) | blue

if __name__ == "__main__":
    rospy.loginfo_once("Starting MiRo Kinematics node ...")
    miroRobot = MiRoKinematics()
    #miroRobot.verbose = True
    #miroRobot.moveHeadCords()
    #miroRobot.subCords()
    #miroRobot.moveHead(None, None)
    #miroRobot.subHasFallen()
    miroRobot.respondFallen()

    #testing if I can control the wheels from here or not 

    """
    while not rospy.core.is_shutdown():
        #miroRobot.residentOkayHaptic()
        miroRobot.turnLEDOn(np.array([255,0,0]), 255)
        #miroRobot.turnLEDOff()
        time.sleep(0.02)
    """

    #disconnecting from robot
    rospy.on_shutdown(miroRobot.cleanUp)
    RobotInterface.disconnect


