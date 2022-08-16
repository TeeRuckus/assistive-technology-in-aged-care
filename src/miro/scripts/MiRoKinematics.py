import rospy
from fallen_analyser.msg import coords
NODE_NAME = "MiRoKinematics"

class MiRoKinematics:

    def __init__(self):
        self.__verbose = False

        rospy.init_node(NODE_NAME, anonymous=True)

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

    def callback_coords(self, data):
        """
        ASSERTION:
        """

        rospy.loginfo(rospy.get_caller_id() + " x Cords: %s", data.xCord.data)
        rospy.loginfo(rospy.get_caller_id() + " y Cords: %s\n", data.yCord.data)


    def subCords(self):
        """
        ASSERTION: Will get coordinate data from fallen_analyser node, and
        MiRo will rotate head to location if in work spac
        """
        rospy.Subscriber("resident/coords/", coords, self.callback_coords)
        #stop node from being excited until node has being stopped
        rospy.spin()


if __name__ == "__main__":
    miro = MiRoKinematics()
    miro.subCords()


