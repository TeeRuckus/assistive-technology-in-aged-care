import rospy
import miro2 as miro
import os
import numpy as np

from fallen_analyser.msg import coords
from std_msgs.msg import UInt8, UInt16, UInt32, Float32MultiArray, UInt16MultiArray, UInt32MultiArray
import geometry_msgs
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import JointState, Imu

NODE_NAME = "MiRoKinematics"

class MiRoKinematics:

    def __init__(self):
        self.__verbose = False
        self.__kinHead = None

        rospy.init_node(NODE_NAME, anonymous=True)

        topicBaseName = "/" + os.getenv("MIRO_ROBOT_NAME")
        topic = topicBaseName + "/control/kinematic_joints"
        self.__pubKin = rospy.Publisher(topic, JointState, queue_size=0)

    def wiggle(self, v, n, m):
        """
        ASSERTION: Wiggles MiRo's tail within the tail's work space
        """
        v = v + float(n) / float(m)
        if v > 2.0:
            v -= 2.0
        elif v > 1.0:
            v = 2.0 - v
        return v

    def moveHead(self, xCord, yCord):
        """
        ASSERTION: Moves head to given xCord and yCord, if points are wthin
        MiRo's workspace
        """

        #creating joint space message to move head
        msgKin = JointState()
        msgKin.position = [0.0, np.radians(30.0), 0.0, 0.0]

        while not rospy.core.is_shutdown():
            #move head to the down position
            msgKin.position[1] = np.radians(30.0)
            msgKin.position[2] = np.radians(0.0)
            msgKin.position[3] = np.radians(0.0)


            self.__pubKin.publish(msgKin)

    def callbackCoords(self, data):
        """
        ASSERTION:
        """

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
        rospy.spin()


if __name__ == "__main__":
    miro = MiRoKinematics()
    #miro.subCords()
    miro.moveHead(None, None)


