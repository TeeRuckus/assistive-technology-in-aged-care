import rospy
import miro2 as miro
import os
import numpy as np
import math
import time


from fallen_analyser.msg import coords
from std_msgs.msg import UInt8, UInt16, UInt32, Float32MultiArray, UInt16MultiArray, UInt32MultiArray
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import JointState, Imu
from Errors import *
#from enum import Enum

NODE_NAME = "MiRoKinematics"
BUFFER_MAX = 2

#TODO: when you will initialise the package, you'll need to make sure that MiRo's eyelids are going to be open
#TODO: When the head is being moved, you want to ignore the incoming coordinates from the pose_fallen node
#TODO: you'll need to transform the use of lists into using arrays for faster access time


class MiRoKinematics:

    def __init__(self):
        self.__verbose = False
        self.__headMoving = False
        self.__kinHead = JointState()
        self.__kinHead.position = [0.0, math.radians(34.0), 0.0, 0.0]
        self.__startTime = 0.0
        #self.__kinHead.name = ["TILT", "LIFT", "YAW", "PITCH"]
        self.__facePos = np.zeros(2)

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

        while not rospy.core.is_shutdown():
            if self.__verbose:
                rospy.loginfo_once("Moving head given subscribed coordinates")

            self.subCords()
            timeHeadDelay = time.time() - self.__startTime


            #don't want to change the pose wile the head is already moving
            if abs(self.__facePos[0]) > 100 and self.__headMoving:
                self.__kinHead.position[miro.constants.JOINT_YAW] += \
                self.moveHead2XCoord(self.__facePos[0])
                self.__headMoving = True
                self.__startTime = time.time()


            #restricting movement to one direction for now
            """
            if abs(self.__facePos[1] > 30:
                self.__kinHead.position[miro.constants.JOINT_LIFT] += \
                self.moveHead2YCoord(self.__facePos[1])

            """

            self.checkHeadWorkSpaceLift()
            self.checkHeadWorkspaceYaw()

            if timeHeadDelay > 1:
                self.__headMoving = False

            if self.__verbose:
                rospy.loginfo("LIFT: %.2f , PITCH: %.2f , YAW: %.2f" %
                        (self.__kinHead.position[miro.constants.JOINT_TILT],
                            self.__kinHead.position[miro.constants.JOINT_PITCH],
                            self.__kinHead.position[miro.constants.JOINT_YAW]))

            #not publish as yet, don't want to break robot
            self.__pubKin.publish(self.__kinHead)

            #processing coordinates at 50Hz. You might need to do 1.5 seconds for this
            time.sleep(0.02)


    def moveHead2XCoord(self, dx):
        """
        ASSERTION:
        """
        return math.radians((dx)*-0.05)


    def moveHead2YCoord(self, dy):
        """
        ASSERTION:
        """
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

            if self.__verbose:
                rospy.loginfo("Miro has reached lift max")

        if self.__kinHead.position[miro.constants.JOINT_LIFT] <= miro.constants.LIFT_RAD_MIN:
            self.__kinHead.position[miro.constants.JOINT_TILT] = \
            miro.constants.LIFT_RAD_MIN + BUFFER_MAX
            workspaceOkay = False

            if self.__verbose:
                rospy.loginfo("Miro has reached lift minimum")

        self.__headMoving = True
        self.__startTime = time.time()

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
            if self.__verbose:
                rospy.loginfo("Miro has reached yaw's max")

        if self.__kinHead.position[miro.constants.JOINT_YAW] <= miro.constants.YAW_RAD_MIN:
            self.__kinHead.position[miro.constants.JOINT_YAW] = \
            miro.constants.YAW_RAD_MIN + BUFFER_MAX

            workspaceOkay = False
            if self.__verbose:
                rospy.loginfo("Miro has reached yaw's minimum")

        self.__headMoving = True
        self.__startTime = time.time()

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

    def callbackCoords(self, data):
        """
        ASSERTION:
        """

        self.__facePos[0] = data.xCord.data
        self.__facePos[1] = data.yCord.data

        if self.__verbose:
            rospy.loginfo(rospy.get_caller_id() + " x Cords: %s", data.xCord.data)
            rospy.loginfo(rospy.get_caller_id() + " y Cords: %s\n", data.yCord.data)


    def callbackKin(self, msg):
        """
        """
        #TODO: you'll need to have some protection, so that if it's not called it will do nothing
        self.__kinHead = msg.position


    def subCords(self):
        """
        ASSERTION: Will get coordinate data from fallen_analyser node, and
        MiRo will rotate head to location if in work spac
        """
        rospy.Subscriber("resident/coords/", coords, self.callbackCoords)
        #stop node from being excited until node has being stopped

    def __validateVerbose(self, inVerbose):
        if type(inVerbose) is not(bool):
            MiRoError("Verbose muse be a boolean either true or false")

        return inVerbose



if __name__ == "__main__":
    miroRobot = MiRoKinematics()
    miroRobot.verbose = True
    miroRobot.moveHeadCords()
    #miroRobot.subCords()
    #miroRobot.moveHead(None, None)


