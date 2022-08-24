import rospy
import miro2 as miro
import os
import numpy as np
import math
import time


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

        rospy.init_node(NODE_NAME, anonymous=True)

        topicBaseName = "/" + os.getenv("MIRO_ROBOT_NAME")

        topic = topicBaseName + "/control/kinematic_joints"
        self.__pubKin = rospy.Publisher(topic, JointState, queue_size=0)

        topic = topicBaseName + "/control/cmd_vel"
        self.__pubWheels = rospy.Publisher(topic, TwistStamped, queue_size=0)

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

            self.subCords()
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

        while not rospy.core.is_shutdown():
            miroRobot.subHasFallen()
            rospy.loginfo("RESPOND FALLEN")

            if self.__fallenState:
                #activating the wheels to drive forward
                msgWheels = TwistStamped()
                #this is the maximum forward speed of MiRo
                msgWheels.twist.linear.x = miro.constants.WHEEL_MAX_SPEED_M_PER_S
                msgWheels.twist.angular.z = 0.0
                self.__pubWheels.publish(msgWheels)

            time.sleep(1)


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

    def wagTail(self):
        """
        ASSERTION: Wags MiRo's tail for a set time to communicate happiness.
        """
        #TODO: come back and finish this  off

        if self.__verbose:
            rospy.loginfo("Wagging MiRo's tail")

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

    def subCords(self):
        """
        ASSERTION: Will subscribe to the topic resident/coords/
        """
        rospy.Subscriber("resident/coords/", coords, self.callbackCoords)
        #stop node from being excited until node has being stopped

    def subHasFallen(self):
        """
        ASSERTION: Will subscribe to the topic resident/fallen/
        """
        rospy.Subscriber("resident/fallen/", Bool, self.callBackHasFallen)
        #rospy.spin()

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

        rospy.loginfo("fallen state %s " % self.__fallenState)

    def callbackKin(self, msg):
        """
        """
        #TODO: you'll need to have some protection, so that if it's not called it will do nothing
        self.__kinHead = msg.position


    def __validateVerbose(self, inVerbose):
        if type(inVerbose) is not(bool):
            MiRoError("Verbose muse be a boolean either true or false")

        return inVerbose



if __name__ == "__main__":
    rospy.loginfo_once("Starting MiRo Kinematics node ...")
    miroRobot = MiRoKinematics()
    #miroRobot.verbose = True
    #miroRobot.moveHeadCords()
    #miroRobot.subCords()
    #miroRobot.moveHead(None, None)
    #miroRobot.subHasFallen()
    miroRobot.respondFallen()

    #disconnecting from robot
    RobotInterface.disconnect


